#include "SystemModel.hpp"

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "mower_msgs/AbsolutePose.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "mower_positioning_core.h"
#include "mower_msgs/WheelTick.h"
#include "mower_positioning/KalmanState.h"
#include "mower_positioning/GPSControlSrv.h"
#include "mower_positioning/SetPoseSrv.h"
#include <std_msgs/Float64.h> // NEW: For debugging vr

ros::Publisher odometry_pub;
ros::Publisher xbot_absolute_pose_pub;

// Debug Publishers
ros::Publisher kalman_state;
ros::Publisher dbg_expected_motion_vector;
// NEW: Debug angular velocity
ros::Publisher debug_vr_pub;

// The kalman filters
mower::positioning::mower_positioning_core core{};

// True, if we don't want to do gyro calibration on launch
bool skip_gyro_calibration;

// True, if we have wheel ticks (i.e. last_ticks is valid)
bool has_ticks;
mower_msgs::WheelTick last_ticks;
bool has_gps;
mower_msgs::AbsolutePose last_gps;

// True, if last_imu is valid and gyro_offset is valid
bool has_gyro;
sensor_msgs::Imu last_imu;
ros::Time gyro_calibration_start;
double gyro_offset;
int gyro_offset_samples;

bool gps_enabled = true;

// Current speed calculated by wheel ticks
double vx = 0.0;

// NEW: Angular velocity for Ackermann model
double vr = 0.0;

// Min speed for motion vector to be fed into kalman filter
double min_speed = 0.0;

// Max position accuracy to allow for GPS updates
double max_gps_accuracy;

// True, if we should publish debug topics (expected motion vector and kalman state)
bool publish_debug;

// Antenna offset (offset between point of rotation and antenna)
double antenna_offset_x, antenna_offset_y;

// NEW: Vehicle type and Ackermann parameters
std::string vehicle_type; // "differential" or "ackermann"
double wheelbase;         // Wheelbase for Ackermann model (meters)

nav_msgs::Odometry odometry;
mower_positioning::KalmanState state_msg;
mower_msgs::AbsolutePose xb_absolute_pose_msg;

int valid_gps_samples = 0;
int gps_outlier_count = 0;
int gps_message_throttle=1;

ros::Time last_gps_time(0.0);

void onImu(const sensor_msgs::Imu::ConstPtr &msg) {
    if(!has_gyro) {
        if(!skip_gyro_calibration) {
            if (gyro_offset_samples == 0) {
                ROS_INFO_STREAM("Started gyro calibration");
                gyro_calibration_start = msg->header.stamp;
                gyro_offset = 0;
            }
            gyro_offset += msg->angular_velocity.z;
            gyro_offset_samples++;
            if ((msg->header.stamp - gyro_calibration_start).toSec() < 5) {
                last_imu = *msg;
                return;
            }
            has_gyro = true;
            if (gyro_offset_samples > 0) {
                gyro_offset /= gyro_offset_samples;
            } else {
                gyro_offset = 0;
            }
            gyro_offset_samples = 0;
            ROS_INFO_STREAM("Calibrated gyro offset: " << gyro_offset);
        } else {
            ROS_WARN("Skipped gyro calibration");
            has_gyro = true;
            return;
        }
    }

    // MODIFIED: Fuse IMU angular velocity with wheel-based vr for Ackermann
    double imu_vr = msg->angular_velocity.z - gyro_offset;
    double fused_vr = imu_vr;
    if (has_ticks && vehicle_type == "ackermann") {
        // For Ackermann, blend IMU and wheel-based angular velocity
        fused_vr = 0.8 * imu_vr + 0.2 * vr; // Adjustable weights
    }

    // core.predict(vx, msg->angular_velocity.z - gyro_offset, (msg->header.stamp - last_imu.header.stamp).toSec());
    // auto x = core.updateSpeed(vx, msg->angular_velocity.z - gyro_offset,0.01);

    // MODIFIED: Use fused_vr for prediction
    core.predict(vx, fused_vr, (msg->header.stamp - last_imu.header.stamp).toSec());
    auto x = core.updateSpeed(vx, fused_vr, 0.01);

    odometry.header.stamp = ros::Time::now();
    odometry.header.seq++;
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = x.x_pos();
    odometry.pose.pose.position.y = x.y_pos();
    odometry.pose.pose.position.z = 0;
    tf2::Quaternion q(0.0, 0.0, x.theta());
    odometry.pose.pose.orientation = tf2::toMsg(q);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header = odometry.header;
    odom_trans.child_frame_id = odometry.child_frame_id;
    odom_trans.transform.translation.x = odometry.pose.pose.position.x;
    odom_trans.transform.translation.y = odometry.pose.pose.position.y;
    odom_trans.transform.translation.z = odometry.pose.pose.position.z;
    odom_trans.transform.rotation = odometry.pose.pose.orientation;

    static tf2_ros::TransformBroadcaster transform_broadcaster;
    transform_broadcaster.sendTransform(odom_trans);

    if(publish_debug) {
        auto state = core.getState();
        state_msg.x = state.x();
        state_msg.y = state.y();
        state_msg.theta = state.theta();
        state_msg.vx = state.vx();
        state_msg.vr = state.vr();

        kalman_state.publish(state_msg);

        // NEW: Publish vr for debugging
        std_msgs::Float64 vr_msg;
        vr_msg.data = fused_vr;
        debug_vr_pub.publish(vr_msg);
    }

    odometry_pub.publish(odometry);

    xb_absolute_pose_msg.header = odometry.header;
    xb_absolute_pose_msg.sensor_stamp = 0;
    xb_absolute_pose_msg.received_stamp = 0;
    xb_absolute_pose_msg.source = mower_msgs::AbsolutePose::SOURCE_SENSOR_FUSION;
    xb_absolute_pose_msg.flags = mower_msgs::AbsolutePose::FLAG_SENSOR_FUSION_DEAD_RECKONING;

    xb_absolute_pose_msg.orientation_valid = true;
    // TODO: send motion vector
    xb_absolute_pose_msg.motion_vector_valid = false;
    // TODO: set real value from kalman filter, not the one from the GPS.
    if(has_gps) {
        xb_absolute_pose_msg.position_accuracy = last_gps.position_accuracy;
    } else {
        xb_absolute_pose_msg.position_accuracy = 999;
    }
    if((ros::Time::now() - last_gps_time).toSec() < 10.0) {
        xb_absolute_pose_msg.flags |= mower_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE;
    } else {
        // on GPS timeout, we set accuracy to 0.
        xb_absolute_pose_msg.position_accuracy = 999;
    }
    // TODO: set real value
    xb_absolute_pose_msg.orientation_accuracy = 0.01;
    xb_absolute_pose_msg.pose = odometry.pose;
    xb_absolute_pose_msg.vehicle_heading = x.theta();
    xb_absolute_pose_msg.motion_heading = x.theta();

    xbot_absolute_pose_pub.publish(xb_absolute_pose_msg);


    last_imu = *msg;
}

void onWheelTicks(const mower_msgs::WheelTick::ConstPtr &msg) {
    if(!has_ticks) {
        last_ticks = *msg;
        has_ticks = true;
        return;
    }
    double dt = (msg->stamp - last_ticks.stamp).toSec();

    double d_wheel_l = (double) (msg->wheel_ticks_rl - last_ticks.wheel_ticks_rl) * (1.0 /(double)msg->wheel_tick_factor);
    double d_wheel_r = (double) (msg->wheel_ticks_rr - last_ticks.wheel_ticks_rr) * (1.0 /(double)msg->wheel_tick_factor);

    if(msg->wheel_direction_rl) {
        d_wheel_l *= -1.0;
    }
    if(msg->wheel_direction_rr) {
        d_wheel_r *= -1.0;
    }

    // MODIFIED: Handle wheel ticks based on vehicle type
    if (vehicle_type == "differential") {
        // Differential drive: average wheel speeds
        double d_ticks = (d_wheel_l + d_wheel_r) / 2.0;
        vx = d_ticks / dt;
        vr = 0.0; // Rely on IMU for angular velocity
    } else if (vehicle_type == "ackermann") {
        // Ackermann drive: average wheel speeds for vx, use steering angle for vr
        double v_left = d_wheel_l / dt;
        double v_right = d_wheel_r / dt;
        vx = (v_left + v_right) / 2.0;

        double steering_angle = msg->steering_angle;
        vr = 0.0;
        if (std::abs(steering_angle) > 1e-3) { // Avoid division by zero
            vr = vx * std::tan(steering_angle) / wheelbase;
        }

        // NEW: Optional validation of wheel speed difference
        double speed_diff = std::abs(v_right - v_left);
        if (speed_diff > 0.5 * std::abs(vx)) { // Threshold for inconsistency
            ROS_WARN_STREAM("Large wheel speed difference detected: " << speed_diff << " vs avg vx: " << vx);
        }
    } else {
        ROS_ERROR_STREAM("Unknown vehicle type: " << vehicle_type);
        return;
    }
    // double d_ticks = (d_wheel_l + d_wheel_r) / 2.0;
    // vx = d_ticks / dt;

    last_ticks = *msg;
}

bool setGpsState(mower_positioning::GPSControlSrvRequest &req, mower_positioning::GPSControlSrvResponse &res) {
    gps_enabled = req.gps_enabled;
    return true;
}

bool setPose(mower_positioning::SetPoseSrvRequest &req, mower_positioning::SetPoseSrvResponse &res) {
    tf2::Quaternion q;
    tf2::fromMsg(req.robot_pose.orientation, q);


    tf2::Matrix3x3 m(q);
    double unused1, unused2, yaw;

    m.getRPY(unused1, unused2, yaw);
    core.setState(req.robot_pose.position.x, req.robot_pose.position.y, yaw,0,0);
    return true;
}

void onPose(const mower_msgs::AbsolutePose::ConstPtr &msg) {
    // 打印所有 msg 数据
    // std::stringstream ss;
    // ss << "Received AbsolutePose message:\n"
    //    << "  Header:\n"
    //    << "    stamp: " << msg->header.stamp << "\n"
    //    << "    frame_id: " << msg->header.frame_id << "\n"
    //    << "  Pose:\n"
    //    << "    position: (" << msg->pose.pose.position.x << ", "
    //                         << msg->pose.pose.position.y << ", "
    //                         << msg->pose.pose.position.z << ")\n"
    //    << "    orientation: (" << msg->pose.pose.orientation.x << ", "
    //                           << msg->pose.pose.orientation.y << ", "
    //                           << msg->pose.pose.orientation.z << ", "
    //                           << msg->pose.pose.orientation.w << ")\n"
    //    << "  Position Accuracy: " << msg->position_accuracy << "\n"
    //    << "  Flags: " << std::hex << "0x" << msg->flags << std::dec << "\n"
    //    << "  Motion Vector: (" << msg->motion_vector.x << ", "
    //                             << msg->motion_vector.y << ", "
    //                             << msg->motion_vector.z << ")";
    // ROS_INFO_STREAM(ss.str());

    if(!gps_enabled) {        
        ROS_INFO_STREAM_THROTTLE(gps_message_throttle, "dropping GPS update, since gps_enabled = false.");
        return;
    }
    // TODO fuse with high covariance?
    if((msg->flags & (mower_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED)) == 0) {
        ROS_INFO_STREAM_THROTTLE(1, "Dropped GPS update, since it's not RTK Fixed");
        return;
    }

    if(msg->position_accuracy > max_gps_accuracy) {
        ROS_INFO_STREAM_THROTTLE(1, "Dropped GPS update, since it's not accurate enough. Accuracy was: " << msg->position_accuracy << ", limit is:" << max_gps_accuracy);
        return;
    }

    double time_since_last_gps = (ros::Time::now() - last_gps_time).toSec();
    if (time_since_last_gps > 5.0) {
        ROS_WARN_STREAM("Last GPS was " << time_since_last_gps << " seconds ago.");
        has_gps = false;
        valid_gps_samples = 0;
        gps_outlier_count = 0;
        last_gps = *msg;
        // we have GPS for next time
        last_gps_time = ros::Time::now();
        return;
    }

    tf2::Vector3 gps_pos(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    tf2::Vector3 last_gps_pos(last_gps.pose.pose.position.x,last_gps.pose.pose.position.y,last_gps.pose.pose.position.z);

    double distance_to_last_gps = (last_gps_pos - gps_pos).length();

    if (distance_to_last_gps < 5.0) {
        // inlier, we treat it normally
        // store the gps as last
        last_gps = *msg;
        last_gps_time = ros::Time::now();

        gps_outlier_count = 0;
        valid_gps_samples++;

        if (!has_gps && valid_gps_samples > 10) {
            ROS_INFO_STREAM("GPS data now valid");
            ROS_INFO_STREAM("First GPS data, moving kalman filter to " << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y);
            // we don't even have gps yet, set odometry to first estimate
            core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.001);

            has_gps = true;
        } else if (has_gps) {
            // gps was valid before, we apply the filter
            core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, 500.0);
            if(publish_debug) {
                auto m = core.om2.h(core.ekf.getState());
                geometry_msgs::Vector3 dbg;
                dbg.x = m.vx();
                dbg.y = m.vy();
                dbg_expected_motion_vector.publish(dbg);
            }
            if(std::sqrt(std::pow(msg->motion_vector.x, 2)+std::pow(msg->motion_vector.y, 2)) >= min_speed) {
                core.updateOrientation2(msg->motion_vector.x, msg->motion_vector.y, 10000.0);
            }
        }
    } else {
        ROS_WARN_STREAM("GPS outlier found. Distance was: " << distance_to_last_gps);
        gps_outlier_count++;
        // ~10 sec
        if (gps_outlier_count > 10) {
            ROS_ERROR_STREAM("too many outliers, assuming that the current gps value is valid.");
            // store the gps as last
            last_gps = *msg;
            last_gps_time = ros::Time::now();
            has_gps = false;

            valid_gps_samples = 0;
            gps_outlier_count = 0;
        }
    }
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "mower_positioning");
    
    has_gps = false;
    gps_enabled = true;
    vx = 0.0;
    vr = 0.0; // NEW: Initialize vr
    has_gyro = false;
    has_ticks = false;
    gyro_offset = 0;
    gyro_offset_samples = 0;

    valid_gps_samples = 0;
    gps_outlier_count = 0;

    antenna_offset_x = antenna_offset_y = 0;

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    ros::ServiceServer gps_service = n.advertiseService("mower_positioning/set_gps_state", setGpsState);
    ros::ServiceServer pose_service = n.advertiseService("mower_positioning/set_robot_pose", setPose);

    // MODIFIED: Load vehicle type and Ackermann parameters
    paramNh.param("vehicle_type", vehicle_type, std::string("differential"));
    paramNh.param("wheelbase", wheelbase, 1.0); // Default wheelbase 1.0m
    paramNh.param("skip_gyro_calibration", skip_gyro_calibration, false);
    paramNh.param("gyro_offset", gyro_offset, 0.0);
    paramNh.param("min_speed", min_speed, 0.01);
    paramNh.param("max_gps_accuracy", max_gps_accuracy, 0.1);
    paramNh.param("debug", publish_debug, false);
    paramNh.param("antenna_offset_x", antenna_offset_x, 0.0);
    paramNh.param("antenna_offset_y", antenna_offset_y, 0.0);
    paramNh.param("gps_message_throttle", gps_message_throttle, 1);

    // NEW: Validate vehicle type
    if (vehicle_type != "differential" && vehicle_type != "ackermann") {
        ROS_ERROR_STREAM("Invalid vehicle_type: " << vehicle_type << ". Using default: differential");
        vehicle_type = "differential";
    }
    ROS_INFO_STREAM("Vehicle type: " << vehicle_type);
    if (vehicle_type == "ackermann") {
        ROS_INFO_STREAM("Ackermann wheelbase: " << wheelbase);
    }

    core.setAntennaOffset(antenna_offset_x, antenna_offset_y);

    ROS_INFO_STREAM("Antenna offset: " << antenna_offset_x << ", " << antenna_offset_y);

    if(gyro_offset != 0.0 && skip_gyro_calibration) {
        ROS_WARN_STREAM("Using gyro offset of: " << gyro_offset);
    }

    odometry_pub = paramNh.advertise<nav_msgs::Odometry>("odom_out", 50);
    xbot_absolute_pose_pub = paramNh.advertise<mower_msgs::AbsolutePose>("xb_pose_out", 50);
    if(publish_debug) {
        dbg_expected_motion_vector = paramNh.advertise<geometry_msgs::Vector3>("debug_expected_motion_vector", 50);
        kalman_state = paramNh.advertise<mower_positioning::KalmanState>("kalman_state", 50);
        debug_vr_pub = paramNh.advertise<std_msgs::Float64>("debug_vr", 10); // NEW: Debug vr
    }

    // <remap from="~imu_in" to="/imu/data_raw"/> /imu/data_raw published by base
    ros::Subscriber imu_sub = paramNh.subscribe("imu_in", 10, onImu);
    ros::Subscriber pose_sub = paramNh.subscribe("xb_pose_in", 10, onPose);
    ros::Subscriber wheel_tick_sub = paramNh.subscribe("wheel_ticks_in", 10, onWheelTicks);

    ros::spin();
    ros::shutdown();
    return 0;
}
