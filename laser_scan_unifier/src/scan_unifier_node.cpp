/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "laser_scan_unifier/scan_unifier_node.hpp"


// Constructor
ScanUnifierNode::ScanUnifierNode() 
  : Node("scan_unifier_node")
{
  auto logger_ = this->get_logger();
  RCLCPP_DEBUG(logger_, "Initializing scan_unifier_node...");
  
  getParams();

  synchronizer2_ = NULL;
  synchronizer3_ = NULL;
  synchronizer4_ = NULL;

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publisher
  topicPub_LaserUnified_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_unified", 1);

  // Subscribe to Laserscan topics
  auto subscriber_options = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  subscriber_options.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  for(size_t i = 0; i < params_.number_of_scans; i++){
    auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, params_.input_topic_names[i], subscriber_options.get_rmw_qos_profile());
    message_filter_subscribers_.push_back(sub);
  }

  // Initialize message_filters::Synchronizer with the right constructor for the choosen number of inputs.
  switch (params_.number_of_scans)
  {
    case 2:
    { 
      synchronizer2_ = std::make_shared<message_filters::Synchronizer<Sync2Policy>>(Sync2Policy(2), 
        *message_filter_subscribers_[0], 
        *message_filter_subscribers_[1]);
      synchronizer2_->setInterMessageLowerBound(0, rclcpp::Duration::from_seconds(params_.inter_message_lower_bound));
      synchronizer2_->setInterMessageLowerBound(1, rclcpp::Duration::from_seconds(params_.inter_message_lower_bound));
      synchronizer2_->registerCallback(&ScanUnifierNode::sync2FilterCallback, this);
      break;
    }
    case 3:
    { 
      synchronizer3_ = std::make_shared<message_filters::Synchronizer<Sync3Policy>>(Sync3Policy(3), 
        *message_filter_subscribers_[0], 
        *message_filter_subscribers_[1], 
        *message_filter_subscribers_[2]);
      synchronizer3_->setInterMessageLowerBound(0, rclcpp::Duration::from_seconds(params_.inter_message_lower_bound));
      synchronizer3_->setInterMessageLowerBound(1, rclcpp::Duration::from_seconds(params_.inter_message_lower_bound));
      synchronizer3_->setInterMessageLowerBound(2, rclcpp::Duration::from_seconds(params_.inter_message_lower_bound));
      synchronizer3_->registerCallback(&ScanUnifierNode::sync3FilterCallback, this);
      break;
    }
    case 4:
    { 
      synchronizer4_ = std::make_shared<message_filters::Synchronizer<Sync4Policy>>(Sync4Policy(10), 
        *message_filter_subscribers_[0], 
        *message_filter_subscribers_[1], 
        *message_filter_subscribers_[2], 
        *message_filter_subscribers_[3]);
      synchronizer4_->setInterMessageLowerBound(0, rclcpp::Duration::from_seconds(params_.inter_message_lower_bound));
      synchronizer4_->setInterMessageLowerBound(1, rclcpp::Duration::from_seconds(params_.inter_message_lower_bound));
      synchronizer4_->setInterMessageLowerBound(2, rclcpp::Duration::from_seconds(params_.inter_message_lower_bound));
      synchronizer4_->setInterMessageLowerBound(3, rclcpp::Duration::from_seconds(params_.inter_message_lower_bound));
      synchronizer4_->registerCallback(&ScanUnifierNode::sync4FilterCallback, this);
      break;
    }
    default:
      RCLCPP_ERROR_STREAM(logger_, params_.number_of_scans << " topics have been set as input, but scan_unifier supports between 2 and 4 topics.");
      return;
  }

  RCLCPP_DEBUG(logger_, "Initialization complete.");
  rclcpp::sleep_for(std::chrono::seconds(1));
}


bool ScanUnifierNode::unifyLaserScans(const std::vector<sensor_msgs::msg::LaserScan::SharedPtr>& current_scans,
                                      sensor_msgs::msg::LaserScan& unified_scan)
{
  if(current_scans.empty()) 
  {
    RCLCPP_DEBUG(this->get_logger(), "No scans to unify.");
    return false;
  }

  if (vec_cloud_.size() != params_.number_of_scans)
  {
    vec_cloud_.resize(params_.number_of_scans);
  }
  auto logger_ = this->get_logger();

  
  RCLCPP_DEBUG(logger_, "Starting conversion...");

  for(size_t i=0; i < params_.number_of_scans; i++)
  {
    RCLCPP_DEBUG(logger_, " - project to PointCloud2");
    projector_.projectLaser(*current_scans[i], vec_cloud_[i]);

    // Transform cloud if necessary
    if (!params_.output_frame.empty() &&vec_cloud_[i].header.frame_id != params_.output_frame) {
      try 
      {
        auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        tf_buffer_->transform(vec_cloud_[i], *cloud, params_.output_frame, tf2::durationFromSec(0.1)); //make into parameter
        vec_cloud_[i] = *cloud;
      } 
      catch (tf2::TransformException & ex) 
      {
        RCLCPP_DEBUG(logger_, " - PointCloud2 transform failure: %s", ex.what());
        return false;
      }
    }
    
  }

  RCLCPP_DEBUG(logger_, "... Complete! Unifying scans...");
  RCLCPP_DEBUG(logger_, " - Creating message header");
  unified_scan.header = current_scans.front()->header;
  unified_scan.header.frame_id = params_.output_frame;
  unified_scan.angle_increment = std::abs(current_scans.front()->angle_increment);
  unified_scan.angle_min = static_cast<float>(-M_PI + unified_scan.angle_increment*0.01);
  unified_scan.angle_max = static_cast<float>(M_PI - unified_scan.angle_increment*0.01);
  unified_scan.time_increment = current_scans.front()->time_increment;
  unified_scan.scan_time = current_scans.front()->scan_time;
  unified_scan.range_min = current_scans.front()->range_min;
  unified_scan.range_max = current_scans.front()->range_max;
  //default values (ranges: range_max, intensities: 0) are used to better reflect the driver behavior
  //there "phantom" data has values > range_max
  //but those values are removed during projection to pointcloud
  unified_scan.ranges.resize(round((unified_scan.angle_max - unified_scan.angle_min) / unified_scan.angle_increment) + 1.0, unified_scan.range_max);
  unified_scan.intensities.resize(round((unified_scan.angle_max - unified_scan.angle_min) / unified_scan.angle_increment+ 1.0), 0.0);

  // now unify all Scans
  for(size_t j = 0; j < params_.number_of_scans; j++)
  { 
    // Iterating over PointCloud2 point xyz values
    sensor_msgs::PointCloud2Iterator<float> iterX(vec_cloud_[j], "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(vec_cloud_[j], "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(vec_cloud_[j], "z");

    for (; iterX != iterX.end(); ++iterX, ++iterY, ++iterZ)
    {
      const float& x = *iterX;
      const float& y = *iterY;
      const float& z = *iterZ;

      if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
      {
        RCLCPP_DEBUG(logger_, " - Rejected for nan in point(%f, %f, %f)\n", x, y, z);
        continue;
      }
      double angle = atan2(y, x);// + M_PI/2;
      if (angle < unified_scan.angle_min || angle > unified_scan.angle_max)
      {
        RCLCPP_DEBUG(logger_, " - Rejected for angle %f not in range (%f, %f)\n", angle, unified_scan.angle_min, unified_scan.angle_max);
        continue;
      }
      int index = std::floor(0.5 + (angle - unified_scan.angle_min) / unified_scan.angle_increment);
      if(index<0 || index>=unified_scan.ranges.size()) continue;
      double range_sq = y*y+x*x;
      //printf ("index xyz( %f %f %f) angle %f index %d range %f\n", x, y, z, angle, index, sqrt(range_sq));
      if ((sqrt(range_sq) <= unified_scan.ranges[index]))
      {
        // use the nearest reflection point of all scans for unified scan
        unified_scan.ranges[index] = sqrt(range_sq);
      }
    }
  }
    
  return true;
}

void ScanUnifierNode::publish(sensor_msgs::msg::LaserScan& unified_scan)
{
  RCLCPP_DEBUG(this->get_logger(), "Publishing unified scan.");
  topicPub_LaserUnified_->publish(unified_scan);
  return;
}

void ScanUnifierNode::sync2FilterCallback(const sensor_msgs::msg::LaserScan::SharedPtr& scan1,
                                          const sensor_msgs::msg::LaserScan::SharedPtr& scan2)
{
  std::vector<sensor_msgs::msg::LaserScan::SharedPtr> current_scans;
  current_scans.push_back(scan1);
  current_scans.push_back(scan2);

  auto unified_scan = sensor_msgs::msg::LaserScan();

  if (!unifyLaserScans(current_scans, unified_scan)) return;
  publish(unified_scan);
  return;
}

void ScanUnifierNode::sync3FilterCallback(const sensor_msgs::msg::LaserScan::SharedPtr& scan1,
                                          const sensor_msgs::msg::LaserScan::SharedPtr& scan2,
                                          const sensor_msgs::msg::LaserScan::SharedPtr& scan3)
{
  std::vector<sensor_msgs::msg::LaserScan::SharedPtr> current_scans;
  current_scans.push_back(scan1);
  current_scans.push_back(scan2);
  current_scans.push_back(scan3);

  auto unified_scan = sensor_msgs::msg::LaserScan();

  if (!unifyLaserScans(current_scans, unified_scan)) return;
  publish(unified_scan);
  return;
}

void ScanUnifierNode::sync4FilterCallback(const sensor_msgs::msg::LaserScan::SharedPtr& scan1,
                                          const sensor_msgs::msg::LaserScan::SharedPtr& scan2,
                                          const sensor_msgs::msg::LaserScan::SharedPtr& scan3,
                                          const sensor_msgs::msg::LaserScan::SharedPtr& scan4)
{
  std::vector<sensor_msgs::msg::LaserScan::SharedPtr> current_scans;
  current_scans.push_back(scan1);
  current_scans.push_back(scan2);
  current_scans.push_back(scan3);
  current_scans.push_back(scan4);

  auto unified_scan = sensor_msgs::msg::LaserScan();

  if (!unifyLaserScans(current_scans, unified_scan)) return;
  publish(unified_scan);
  return;
}

void ScanUnifierNode::getParams()
{
  auto logger_ = this->get_logger();
  param_listener_ = std::make_shared<scan_unifier_node::ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  RCLCPP_DEBUG(logger_, "Parameters read. Number of input scans: %ld", params_.number_of_scans);
  RCLCPP_DEBUG(logger_, "Topic names:");
  for (auto &&topic : params_.input_topic_names)
  {
    RCLCPP_DEBUG(logger_, "  /%s", topic.c_str());
  }
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanUnifierNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


