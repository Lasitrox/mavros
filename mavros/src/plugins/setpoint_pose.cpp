/**
 * @brief SetpointPose plugin
 * @file setpoint_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PoseSetpoint.h>

#include <mavros_msgs/SetMavFrame.h>
#include <geographic_msgs/GeoPoseStamped.h>

#include <GeographicLib/Geocentric.hpp>

namespace mavros {
namespace std_plugins {
using mavlink::common::MAV_FRAME;
/**
 * @brief Setpoint position plugin
 *
 * Send setpoint positions to FCU controller.
 */
class SetpointPosePlugin : public plugin::PluginBase,
	private plugin::SetPoseTargetLocalNEDMixin<SetpointPosePlugin> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SetpointPosePlugin() : PluginBase(),
		sp_nh("~setpoint_pose"),
		tf_rate(50.0),
		tf_listen(false)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		// tf params
		sp_nh.param("tf/listen", tf_listen, false);
		sp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		sp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "target_position");
		sp_nh.param("tf/rate_limit", tf_rate, 50.0);


		setpoint_sub = sp_nh.subscribe("pose", 10, &SetpointPosePlugin::setpose_cb, this);
		setpose_sub = sp_nh.subscribe("position", 10, &SetpointPosePlugin::setpoint_cb, this);
		setpose_raw_sub = sp_nh.subscribe("raw", 10, &SetpointPosePlugin::setpoint_raw_cb, this);

		mav_frame_srv = sp_nh.advertiseService("mav_frame", &SetpointPosePlugin::set_mav_frame_cb, this);

		// mav_frame
		std::string mav_frame_str;
		if (!sp_nh.getParam("mav_frame", mav_frame_str)) {
			mav_frame = MAV_FRAME::LOCAL_NED;
		} else {
			mav_frame = utils::mav_frame_from_str(mav_frame_str);
		}
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	friend class SetPoseTargetLocalNEDMixin;

	ros::NodeHandle sp_nh;
	ros::NodeHandle spg_nh;		//!< to get local position and gps coord which are not under sp_h()
	ros::Subscriber setpoint_sub;
	ros::Subscriber setpose_sub;	//!< Global setpoint
	ros::Subscriber setpose_raw_sub;//!< Global setpoint converted to local setpoint
	ros::ServiceServer mav_frame_srv;

	/* Stores current gps state. */
	//sensor_msgs::NavSatFix current_gps_msg;
	Eigen::Vector3d current_gps;		//!< geodetic coordinates LLA
	Eigen::Vector3d current_local_pos;	//!< Current local position in ENU
	uint32_t old_gps_stamp = 0;		//!< old time gps time stamp in [ms], to check if new gps msg is received

	std::string tf_frame_id;
	std::string tf_child_frame_id;

	bool tf_listen;
	double tf_rate;

	MAV_FRAME mav_frame;

	/* -*- callbacks -*- */

	void setpose_cb(const geometry_msgs::PoseStamped::ConstPtr &req)
	{
		using mavlink::common::MAV_FRAME;

		geometry_msgs::Pose pose = req->pose;
		
		Eigen::Vector3d position;
		Eigen::Quaterniond attitude;

		tf::pointMsgToEigen(pose.position, position);
		tf::quaternionMsgToEigen(pose.orientation, attitude);

		//Velocitys and accelerations deactivated
		uint16_t typemask = 1 >> 1 ||
		1 >> 2 ||
		1 >> 4 ||
		1 >> 5;

		set_pose_target_local_ned(req->header.stamp.toNSec() / 1000000, 
			utils::enum_value(mav_frame),
			typemask, 
			ftf::transform_frame_enu_ned(position), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
			ftf::transform_orientation_enu_ned(attitude), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
	}

	void setpoint_cb(const geometry_msgs::PoseStamped::ConstPtr &req)
	{
		using mavlink::common::MAV_FRAME;

		geometry_msgs::Pose pose = req->pose;
		
		Eigen::Vector3d position;
		Eigen::Quaterniond attitude;

		tf::pointMsgToEigen(pose.position, position);
		tf::quaternionMsgToEigen(pose.orientation, attitude);

		//Velocitys and accelerations deactivated, yaw and attitude free for transit
		uint16_t typemask = 1 >> 1 ||
		1 >> 2 ||
		1 >> 4 ||
		1 >> 5 ||
		1 >> 8 ||
		1 >> 9;

		set_pose_target_local_ned(req->header.stamp.toNSec() / 1000000, 
			utils::enum_value(mav_frame),
			typemask, 
			ftf::transform_frame_enu_ned(position), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
			ftf::transform_orientation_enu_ned(attitude), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
	}

	void setpoint_raw_cb(const mavros_msgs::PoseSetpoint::ConstPtr &req)
	{
		using mavlink::common::MAV_FRAME;

		Eigen::Affine3d d;

		geometry_msgs::Pose pose = req->pose;
		geometry_msgs::Twist velocitys = req->velocity;
		geometry_msgs::Accel accelerations = req->acceleration;

		Eigen::Vector3d position;
		Eigen::Vector3d velocity;
		Eigen::Vector3d acceleration;
		Eigen::Quaterniond attitude;
		Eigen::Vector3d angular_rates;
		Eigen::Vector3d angular_acceleration;

		tf::pointMsgToEigen(pose.position, position);
		tf::vectorMsgToEigen(velocitys.linear, velocity);
		tf::vectorMsgToEigen(accelerations.linear, acceleration);
		tf::quaternionMsgToEigen(pose.orientation, attitude);
		tf::vectorMsgToEigen(velocitys.angular, angular_rates);
		tf::vectorMsgToEigen(accelerations.angular, angular_acceleration);

		//All activ
		uint16_t typemask = 0;

		set_pose_target_local_ned(req->header.stamp.toNSec() / 1000000, 
			utils::enum_value(mav_frame),
			typemask, 
			ftf::transform_frame_enu_ned(position), 
			ftf::transform_frame_enu_ned(velocity), 
			ftf::transform_frame_enu_ned(acceleration),
			ftf::transform_orientation_enu_ned(attitude), 
			ftf::transform_frame_enu_ned(angular_rates), 
			ftf::transform_frame_enu_ned(angular_acceleration));
	}

	bool set_mav_frame_cb(mavros_msgs::SetMavFrame::Request &req, mavros_msgs::SetMavFrame::Response &res)
	{
		mav_frame = static_cast<MAV_FRAME>(req.mav_frame);
		const std::string mav_frame_str = utils::to_string(mav_frame);
		sp_nh.setParam("mav_frame", mav_frame_str);
		res.success = true;
		return true;
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointPosePlugin, mavros::plugin::PluginBase)
