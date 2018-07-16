/****************************************************************
 *
 * Copyright (c) 2017
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: lips
 * ROS package name: lips_camera
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Richard Bormann, email:richard.bormann@ipa.fhg.de
 *
 * Date of creation: July 2017
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef __LIPS_CAMERA_H__
#define __LIPS_CAMERA_H__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_broadcaster.h>

//#include <std_srvs/Trigger.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <lips/OpenNI.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class LipsCamera
{
public:
	LipsCamera(ros::NodeHandle& nh);
	~LipsCamera();

	void init();

	void open();

	void close();

	bool isInitialized()
	{
		return initialized_;
	}

	bool isOpen()
	{
		return open_;
	}

	void run();

private:

	bool acquirePointCloud(pcl::PointCloud<pcl::PointXYZ>& point_cloud, const bool process_xyz,
			pcl::PointCloud<pcl::PointXYZRGB>& point_cloud_rgb, const bool process_xyzrgb, cv::Mat& color_image, const bool publish_color_image);

	void fillCameraInfo(sensor_msgs::CameraInfo &camera_info_msg) const;
	void fillCameraInfoRGBD(sensor_msgs::CameraInfo &camera_info_msg) const;

//	bool activateSensor(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
//	bool deactivateSensor(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
//	bool activateSensorOnce(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

	// devices
	openni::Device camera_device_;		///< Openni Camera Device

	// streams
	openni::VideoStream depth_stream_;	///< depth video stream
	openni::VideoStream color_stream_;	///< color video stream

	// parameters
	std::string rgb_frame_id_;		///< the frame id of the color camera
	bool enable_rgbd_processing_;		///< activates RGB-D processing with attached color camera if set true
	bool publish_color_image_;			///< publishes the image of the attached color camera if set true

	// camera status
	bool initialized_;
	bool open_;
	bool sensor_enabled_;		///< additional flag for enabling or disabling sensor processing via service
	bool sensor_enabled_once_;		///< additional flag for enabling sensor processing for one frame via service

	// publishers
	ros::Publisher point_cloud_pub_;	///< point cloud output topic
	ros::Publisher point_cloud_rgb_pub_;	///< point cloud output topic
	ros::Publisher camera_info_pub_;	///< camera info output topic
	ros::Publisher camera_info_rgb_pub_;	///< camera info output topic
	tf::TransformBroadcaster tf_broadcaster_; ///< broadcasts transform from ids_ueye_frame_id_ to ensenso_frame_id_

	// subscribers
	boost::shared_ptr<image_transport::ImageTransport> it_;
	image_transport::Publisher color_image_pub_;

//	// services
//	ros::ServiceServer activate_sensor_server_;		// service server for activating sensor processing and publishing
//	ros::ServiceServer deactivate_sensor_server_;		// service server for deactivating sensor processing and publishing
//	ros::ServiceServer activate_sensor_once_server_;		// service server for activating sensor processing and publishing for one frame

	ros::NodeHandle node_handle_; ///< ROS node handle
	
};

#endif //__LIPS_CAMERA_H__
