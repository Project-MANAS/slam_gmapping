//
// Created by shivesh on 29/10/18.
//

#ifndef SLAM_GMAPPING_SLAM_GMAPPING_H_
#define SLAM_GMAPPING_SLAM_GMAPPING_H_

#include <mutex>
#include <thread>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/utils.h"

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"
#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

class SlamGmapping : public rclcpp::Node{
public:
    SlamGmapping();
    ~SlamGmapping() override;

    void init();
    void startLiveSlam();
    void publishTransform();
    void laserCallback(sensor_msgs::msg::LaserScan::SharedPtr scan);
    void publishLoop(double transform_publish_period);

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr entropy_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr sst_;
    rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr sstm_;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tfl_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_filter_sub_;
    tf2_ros::TransformBroadcaster* tfB_;

    GMapping::GridSlamProcessor* gsp_;
    GMapping::RangeSensor* gsp_laser_;
    // The angles in the laser, going from -x to x (adjustment is made to get the laser between
    // symmetrical bounds as that's what gmapping expects)
    std::vector<double> laser_angles_;
    // The pose, in the original laser frame, of the corresponding centered laser with z facing up
    geometry_msgs::msg::PoseStamped centered_laser_pose_;
    // Depending on the order of the elements in the scan and the orientation of the scan frame,
    // We might need to change the order of the scan
    bool do_reverse_range_;
    unsigned int gsp_laser_beam_count_;
    GMapping::OdometrySensor* gsp_odom_;

    bool got_first_scan_;

    bool got_map_;
    nav_msgs::msg::OccupancyGrid map_;

    rclcpp::Duration map_update_interval_;
    tf2::Transform map_to_odom_;
    std::mutex map_to_odom_mutex_;
    std::mutex map_mutex_;

    int laser_count_;
    int throttle_scans_;

    std::thread* transform_thread_;

    std::string base_frame_;
    std::string laser_frame_;
    std::string map_frame_;
    std::string odom_frame_;

    void updateMap(sensor_msgs::msg::LaserScan::SharedPtr scan);
    bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const rclcpp::Time& t);
    bool initMapper(sensor_msgs::msg::LaserScan::SharedPtr scan);
    bool addScan(sensor_msgs::msg::LaserScan::SharedPtr scan, GMapping::OrientedPoint& gmap_pose);
    double computePoseEntropy();

    // Parameters used by GMapping
    double maxRange_;
    double maxUrange_;
    double maxrange_;
    double minimum_score_;
    double sigma_;
    int kernelSize_;
    double lstep_;
    double astep_;
    int iterations_;
    double lsigma_;
    double ogain_;
    int lskip_;
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linearUpdate_;
    double angularUpdate_;
    double temporalUpdate_;
    double resampleThreshold_;
    int particles_;
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    double delta_;
    double occ_thresh_;
    double llsamplerange_;
    double llsamplestep_;
    double lasamplerange_;
    double lasamplestep_;

    unsigned long int seed_;

    double transform_publish_period_;
    double tf_delay_;
};

#endif //SLAM_GMAPPING_SLAM_GMAPPING_H_
