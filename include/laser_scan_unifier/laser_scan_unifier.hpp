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


#ifndef LASER_SCAN_UNIFIER__SCAN_UNIFIER_NODE_HPP_
#define LASER_SCAN_UNIFIER__SCAN_UNIFIER_NODE_HPP_

//##################
//#### includes ####

// standard includes
#include <thread>
//#include <XmlRpc.h>
#include <math.h>

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/latest_time.h"

// ROS message includes
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// auto-generated by generate_parameter_library
#include "laser_scan_unifier_parameters.hpp"

//####################
//#### node class ####
class ScanUnifierNode : public rclcpp::Node
{
  private:
    // Parameters from ROS
    std::shared_ptr<laser_scan_unifier::ParamListener> param_listener_;
    laser_scan_unifier::Params params_;


    std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>> message_filter_subscribers_;



    void sync2FilterCallback(const sensor_msgs::msg::LaserScan::SharedPtr& first_scanner,
                               const sensor_msgs::msg::LaserScan::SharedPtr& second_scanner);
    void sync3FilterCallback(const sensor_msgs::msg::LaserScan::SharedPtr& first_scanner,
                               const sensor_msgs::msg::LaserScan::SharedPtr& second_scanner,
                               const sensor_msgs::msg::LaserScan::SharedPtr& third_scanner);
    void sync4FilterCallback(const sensor_msgs::msg::LaserScan::SharedPtr& first_scanner,
                               const sensor_msgs::msg::LaserScan::SharedPtr& second_scanner,
                               const sensor_msgs::msg::LaserScan::SharedPtr& third_scanner,
                               const sensor_msgs::msg::LaserScan::SharedPtr& fourth_scanner);


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> ApproximateTimeSync2Policy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> ApproximateTimeSync3Policy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> ApproximateTimeSync4Policy;

    std::shared_ptr<message_filters::Synchronizer<ApproximateTimeSync2Policy>> approximate_time_synchronizer2_;
    std::shared_ptr<message_filters::Synchronizer<ApproximateTimeSync3Policy>> approximate_time_synchronizer3_;
    std::shared_ptr<message_filters::Synchronizer<ApproximateTimeSync4Policy>> approximate_time_synchronizer4_;

    typedef message_filters::sync_policies::LatestTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> LatestTimeSync2Policy;
    typedef message_filters::sync_policies::LatestTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> LatestTimeSync3Policy;
    typedef message_filters::sync_policies::LatestTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> LatestTimeSync4Policy;

    std::shared_ptr<message_filters::Synchronizer<LatestTimeSync2Policy>> latest_time_synchronizer2_;
    std::shared_ptr<message_filters::Synchronizer<LatestTimeSync3Policy>> latest_time_synchronizer3_;
    std::shared_ptr<message_filters::Synchronizer<LatestTimeSync4Policy>> latest_time_synchronizer4_;

  public:
    //constructor
    ScanUnifierNode();
    /* ----------------------------------- */
    /* --------- ROS Variables ----------- */
    /* ----------------------------------- */

    // create publishers
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr topicPub_LaserUnified_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr topicPub_PointCloudUnified_;

    // tf listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // laser geometry projector
    laser_geometry::LaserProjection projector_;


    std::vector<sensor_msgs::msg::PointCloud2> vec_cloud_;
    

    /* ----------------------------------- */
    /* ----------- functions ------------- */
    /* ----------------------------------- */

    /**
     * @function getParams
     * @brief function to load parameters from ros parameter server
     *
     * input: -
     * output: -
     */
     
    void getParams();
    
    
    /**
     * @function unifyLaserScans
     * @brief unify the scan information from all laser scans in vec_laser_struct_
     *
     * input: -
     * output:
     * @param: a laser scan message containing unified information from all scanners
     */
    void publish(sensor_msgs::msg::LaserScan& unified_scan);
    bool unifyLaserScans(const std::vector<sensor_msgs::msg::LaserScan::SharedPtr>& current_scans,
                         sensor_msgs::msg::LaserScan& unified_scan);
                         
};

#endif // LASER_SCAN_UNIFIER__SCAN_UNIFIER_NODE_HPP_
