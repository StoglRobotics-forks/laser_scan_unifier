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
#include "scan_unifier_node_parameters.hpp"

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

// ROS message includes
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> Sync2Policy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> Sync3Policy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> Sync4Policy;
//####################
//#### node class ####
class ScanUnifierNode : public rclcpp::Node
{
  private:
    /** @struct config_struct
     *  @brief This structure holds configuration parameters
     *  @var config_struct::number_input_scans
     *  Member 'number_input_scans' contains number of scanners to subscribe to
     *  @var config_struct::loop_rate
     *  Member 'loop_rate' contains the loop rate of the ros node
     *  @var config_struct::input_scan_topics
     *  Member 'input_scan_topics' contains the names of the input scan topics
     */

    std::shared_ptr<scan_unifier_node::ParamListener> param_listener_;
    scan_unifier_node::Params params_;

    struct config_struct{
      size_t number_input_scans;
      std::vector<std::string> input_scan_topics;
      bool publish_pointcloud = false;
    };
    config_struct config_;
    std::string frame_;

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



    std::shared_ptr<message_filters::Synchronizer<Sync2Policy>> synchronizer2_;    
    std::shared_ptr<message_filters::Synchronizer<Sync3Policy>> synchronizer3_;
    std::shared_ptr<message_filters::Synchronizer<Sync4Policy>> synchronizer4_;


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
