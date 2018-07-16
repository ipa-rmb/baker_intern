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

#include <fstream>
#include <sstream>

#include <ros/package.h>
#include <lips_camera/lips_camera.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>



LipsCamera::LipsCamera(ros::NodeHandle& nh)
	: node_handle_(nh), initialized_ (false), open_(false), sensor_enabled_(true), sensor_enabled_once_(false)
{
	// set parameters
	ROS_DEBUG_STREAM("\n--------------------------\nLipsCamera Parameters:\n--------------------------");
	node_handle_.param("enable_rgbd_processing", enable_rgbd_processing_, false);
	ROS_DEBUG_STREAM("enable_rgbd_processing = " << enable_rgbd_processing_);
	node_handle_.param("publish_color_image", publish_color_image_, false);
	ROS_DEBUG_STREAM("publish_color_image = " << publish_color_image_);
	node_handle_.param<std::string>("rgb_frame_id", rgb_frame_id_, "camera_frame");
	ROS_DEBUG_STREAM("rgb_frame_id = " << rgb_frame_id_);

//	// services
//	activate_sensor_server_ = node_handle_.advertiseService("activate_sensor", &LipsCamera::activateSensor, this);
//	deactivate_sensor_server_ = node_handle_.advertiseService("deactivate_sensor", &LipsCamera::deactivateSensor, this);
//	activate_sensor_once_server_ = node_handle_.advertiseService("activate_sensor_once", &LipsCamera::activateSensorOnce, this);

	it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(node_handle_));
	color_image_pub_ = it_->advertise("color_image", 1);
}

LipsCamera::~LipsCamera()
{
	if (isOpen())
		close();

	if (isInitialized())
	{
		openni::OpenNI::shutdown();
	}
}


void LipsCamera::init()
{
	if (isInitialized())
	{
		return;
	}

	if (openni::STATUS_OK != openni::OpenNI::initialize())
	{
		std::cout << "Error - LipsCamera::init: After initialization: " << openni::OpenNI::getExtendedError() << std::endl;
		return;
	}

	// Set init flag
	initialized_ = true;
}

void LipsCamera::open()
{
	if (!isInitialized())
	{
		std::cerr << "ERROR - LipsCamera::open:" << std::endl;
		std::cerr << "\t ... The camera is not initialized." << std::endl;
		return;
	}

	if (isOpen())
	{
		std::cerr << "INFO - LipsCamera::open:" << std::endl;
		std::cerr << "\t ... The camera is already open." << std::endl;
		return;
	}

	openni::Array<openni::DeviceInfo> info;
	openni::OpenNI::enumerateDevices(&info);
	std::cout << "Found " << info.getSize() << " devices." << std::endl;
	for (int i=0; i<info.getSize(); ++i)
		std::cout << info[i].getName() << ": " << info[i].getUri() << ", " << info[i].getVendor() << std::endl;

	// open camera
	if (openni::STATUS_OK != camera_device_.open(openni::ANY_DEVICE))
	{
		std::cerr << "ERROR - LipsCamera::open:" << std::endl;
		std::cerr << "\t ... Cannot open device: " << openni::OpenNI::getExtendedError() << std::endl;
		return;
	}

	if (openni::STATUS_OK != camera_device_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	{
		std::cerr << "ERROR - LipsCamera::open:" << std::endl;
		std::cerr << "\t ... Cannot set ImageRegistration mode: " << openni::OpenNI::getExtendedError() << std::endl;
	}

	openni::VideoMode mode;
	mode.setResolution(640, 480);
	mode.setFps(30);

	if (openni::STATUS_OK != depth_stream_.create(camera_device_, openni::SENSOR_DEPTH))
	{
		std::cerr << "ERROR - LipsCamera::open:" << std::endl;
		std::cerr << "\t ... Cannot create depth stream on device: " << openni::OpenNI::getExtendedError() << std::endl;
		return;
	}
	else
	{
		mode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
		if (openni::STATUS_OK != depth_stream_.setVideoMode(mode))
		{
			std::cerr << "ERROR - LipsCamera::open:" << std::endl;
			std::cerr << "\t ... Cannot set video mode: " << openni::OpenNI::getExtendedError() << std::endl;
			return;
		}
	}

	//mode.setResolution(1920, 1080);
	if (openni::STATUS_OK != color_stream_.create(camera_device_, openni::SENSOR_COLOR))
	{
		std::cerr << "ERROR - LipsCamera::open:" << std::endl;
		std::cerr << "\t ... Cannot create color stream on device: " << openni::OpenNI::getExtendedError() << std::endl;
		return;
	}
	else
	{
		mode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
		if (openni::STATUS_OK != color_stream_.setVideoMode(mode))
		{
			std::cerr << "ERROR - LipsCamera::open:" << std::endl;
			std::cerr << "\t ... Cannot set color mode: " << openni::OpenNI::getExtendedError() << std::endl;
			return;
		}
	}

	depth_stream_.start();
	color_stream_.start();

	// advertise topic
	point_cloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("pointcloud_out", 1, false);
	camera_info_pub_ = node_handle_.advertise<sensor_msgs::CameraInfo> ("camera_info", 1, false);
	if (enable_rgbd_processing_==true)
	{
		point_cloud_rgb_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("pointcloud_xyzrgb_out", 1, false);
		camera_info_rgb_pub_ = node_handle_.advertise<sensor_msgs::CameraInfo> ("camera_info_rgb", 1, false);
	}

	std::cout << "*************************************************" << std::endl;
	std::cout << "LipsCamera::open: LipsCamera camera device OPEN" << std::endl;
	std::cout << "*************************************************" << std::endl << std::endl;
	open_ = true;
}


void LipsCamera::close()
{
	if (isOpen() == false)
		return;

	std::cout << "INFO - LipsCamera: Closing device..." << std::endl;

	depth_stream_.destroy();
	color_stream_.destroy();

	camera_device_.close();

	open_ = false;
}


void LipsCamera::fillCameraInfo(sensor_msgs::CameraInfo &camera_info_msg) const
{
//	try
//	{
//		int width=0, height=0;
//		ensenso_camera_[itmImages][itmPointMap].getBinaryDataInfo(&width, &height, 0,0,0,0);
//
//		camera_info_msg.header.frame_id = rgb_frame_id_;
//		camera_info_msg.width = width;
//		camera_info_msg.height = height;
//		camera_info_msg.D.resize(5, 0.);
//		camera_info_msg.K.assign(0.);
//		camera_info_msg.K[8]=1.;
//
//		Eigen::Matrix4d reprojection, projection;
//		for(int i=0; i<4; i++)
//			for(int j=0; j<4; j++)
//				reprojection(j,i) = ensenso_camera_[itmCalibration][itmDynamic][itmStereo][itmReprojection][i][j].asDouble();
//
//		projection = reprojection.inverse();
//		projection /= projection(3,2);
//
//		for(int i=0; i<3; i++)
//			for(int j=0; j<4; j++)
//				camera_info_msg.P[i*4+j] = projection(i<2?i:i+1, j);
//
//		camera_info_msg.K[0]=projection(0,0);
//		camera_info_msg.K[2]=projection(0,2);
//		camera_info_msg.K[4]=projection(1,1);
//		camera_info_msg.K[5]=projection(1,2);
//	}
//	catch (NxLibException& ex)
//	{
//		std::cerr << "ERROR - LipsCamera::fillCameraInfo:" << std::endl;
//		std::cerr << ex.getItemPath() << " has error " << ex.getErrorCode() << ": " << ex.getErrorText() << std::endl;
//		return;
//	}
}

void LipsCamera::fillCameraInfoRGBD(sensor_msgs::CameraInfo &camera_info_msg) const
{
//	try
//	{
//		int width = ids_ueye_camera_[itmSensor][itmSize][0].asInt();
//		int height = ids_ueye_camera_[itmSensor][itmSize][1].asInt();
//
//		camera_info_msg.header.frame_id = rgb_frame_id_;
//		camera_info_msg.width = width;
//		camera_info_msg.height = height;
//		camera_info_msg.D.resize(5, 0.);
//		camera_info_msg.K.assign(0.);
//		camera_info_msg.K[8]=1.;
//		camera_info_msg.K[0] = ids_ueye_camera_[itmCalibration][itmCamera][0][0].asDouble();
//		camera_info_msg.K[2] = ids_ueye_camera_[itmCalibration][itmCamera][2][0].asDouble();
//		camera_info_msg.K[4] = ids_ueye_camera_[itmCalibration][itmCamera][1][1].asDouble();
//		camera_info_msg.K[5] = ids_ueye_camera_[itmCalibration][itmCamera][2][1].asDouble();
//
//		camera_info_msg.P.assign(0.);
//		for(int i=0; i<3; i++)
//			for(int j=0; j<3; j++)
//				camera_info_msg.P[i*4+j] = ids_ueye_camera_[itmCalibration][itmCamera][j][i].asDouble();
//	}
//	catch (NxLibException& ex)
//	{
//		std::cerr << "ERROR - LipsCamera::fillCameraInfoRGBD:" << std::endl;
//		std::cerr << ex.getItemPath() << " has error " << ex.getErrorCode() << ": " << ex.getErrorText() << std::endl;
//		return;
//	}
}


void LipsCamera::run()
{
	sensor_msgs::PointCloud2::Ptr point_cloud_out_msg(new sensor_msgs::PointCloud2);
	sensor_msgs::PointCloud2::Ptr point_cloud_rgb_out_msg(new sensor_msgs::PointCloud2);
	
	std_msgs::Header header;
	header.seq = 0;
	
	sensor_msgs::CameraInfo camera_info_msg;
	fillCameraInfo(camera_info_msg);
	sensor_msgs::CameraInfo camera_info_rgb_msg;
	if (enable_rgbd_processing_==true)
		fillCameraInfoRGBD(camera_info_rgb_msg);
	
	ros::Rate rate(60.);	//limit publishing frequency
	while (ros::ok())
	{
		const bool process_xyz = ((sensor_enabled_ || sensor_enabled_once_) && point_cloud_pub_.getNumSubscribers()>0);
		const bool process_xyzrgb = ((sensor_enabled_ || sensor_enabled_once_) && enable_rgbd_processing_==true && point_cloud_rgb_pub_.getNumSubscribers()>0);
		const bool publish_color_image = ((sensor_enabled_ || sensor_enabled_once_) && publish_color_image_==true && color_image_pub_.getNumSubscribers()>0);
		if (process_xyz || process_xyzrgb || publish_color_image)
		{
			// retrieve point clouds
			pcl::PointCloud<pcl::PointXYZ> point_cloud;
			pcl::PointCloud<pcl::PointXYZRGB> point_cloud_rgb;
			cv::Mat color_image;
			std::cout << "1" << std::endl;
			acquirePointCloud(point_cloud, process_xyz, point_cloud_rgb, process_xyzrgb, color_image, publish_color_image);
			std::cout << "2" << std::endl;

			// publish messages
			header.seq++;
			header.stamp = ros::Time::now();

			// publish point cloud xyz
			if (process_xyz)
			{
				std::cout << "3" << std::endl;
				pcl::PCLPointCloud2 pc2;
				pcl::toPCLPointCloud2(point_cloud, pc2);
				pcl_conversions::fromPCL(pc2, *point_cloud_out_msg);
				header.frame_id = rgb_frame_id_;
				point_cloud_out_msg->header = header;
				point_cloud_pub_.publish(point_cloud_out_msg);
				camera_info_msg.header = header;
				camera_info_pub_.publish(camera_info_msg);
				std::cout << "4----------------- xyz published" << std::endl;
			}

			// publish point cloud xyzrgb
			if (process_xyzrgb || publish_color_image)
			{
				if (process_xyzrgb)
				{
					std::cout << "5" << std::endl;
					pcl::PCLPointCloud2 pc2;
					pcl::toPCLPointCloud2(point_cloud_rgb, pc2);
					pcl_conversions::fromPCL(pc2, *point_cloud_rgb_out_msg);
					header.frame_id = rgb_frame_id_;
					point_cloud_rgb_out_msg->header = header;
					point_cloud_rgb_pub_.publish(point_cloud_rgb_out_msg);
					camera_info_rgb_msg.header = header;
					camera_info_rgb_pub_.publish(camera_info_rgb_msg);
					std::cout << "6----------------- xyzrgb published" << std::endl;
				}

				if (publish_color_image)
				{
					std::cout << "7" << std::endl;
					sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_image).toImageMsg();
					color_image_pub_.publish(image_msg);
					std::cout << "8----------------- rgb published" << std::endl;
				}

//				// publish tf
//				try
//				{
//					tf::Transform transform;
//					double x = 0.001*ensenso_camera_[itmLink][itmTranslation][0].asDouble();
//					double y = 0.001*ensenso_camera_[itmLink][itmTranslation][1].asDouble();
//					double z = 0.001*ensenso_camera_[itmLink][itmTranslation][2].asDouble();
//					double alpha = ensenso_camera_[itmLink][itmRotation][itmAngle].asDouble();
//					double rv1 = ensenso_camera_[itmLink][itmRotation][itmAxis][0].asDouble();
//					double rv2 = ensenso_camera_[itmLink][itmRotation][itmAxis][1].asDouble();
//					double rv3 = ensenso_camera_[itmLink][itmRotation][itmAxis][2].asDouble();
//					//std::cout << "Transform ensenso --> ids: " << x << ", " << y << ", " << z << ", " << rv1*sin(0.5*alpha) << ", " << rv2*sin(0.5*alpha) << ", " << rv2*sin(0.5*alpha) << ", " << cos(0.5*alpha) << std::endl;
//					transform.setOrigin(tf::Vector3(x, y, z));
//					transform.setRotation(tf::Quaternion(rv1*sin(0.5*alpha), rv2*sin(0.5*alpha), rv3*sin(0.5*alpha), cos(alpha*0.5)));
//					tf::StampedTransform marker_tf(transform, header.stamp, ensenso_frame_id_, ids_ueye_frame_id_);
//					tf_broadcaster_.sendTransform(marker_tf);
//				}
//				catch (NxLibException& ex)
//				{
//					std::cerr << "ERROR - LipsCamera::run:" << std::endl;
//					std::cerr << ex.getItemPath() << " has error " << ex.getErrorCode() << ": " << ex.getErrorText() << std::endl;
//					continue;
//				}
//				std::cout << "9----------------- tf published" << std::endl;
			}

			// reset flag
			sensor_enabled_once_ = false;
		}

		ros::spinOnce();
		rate.sleep();
	}
}

bool LipsCamera::acquirePointCloud(pcl::PointCloud<pcl::PointXYZ>& point_cloud, const bool process_xyz,
		pcl::PointCloud<pcl::PointXYZRGB>& point_cloud_rgb, const bool process_xyzrgb, cv::Mat& color_image, const bool publish_color_image)
{
	// 1. XYZ point cloud
	if (process_xyz || process_xyzrgb)
	{
		// acquire data from depth camera
		openni::VideoFrameRef depth_frame;
		cv::Mat depth_image;
		if (depth_stream_.isValid())
		{
			if (openni::STATUS_OK == depth_stream_.readFrame(&depth_frame))
			{
				// copy data
				const int xyz_height = depth_frame.getHeight();
				const int xyz_width = depth_frame.getWidth();
				depth_image = cv::Mat(xyz_height, xyz_width, CV_16UC1, (void*)depth_frame.getData());

				// write data into point cloud
				point_cloud.clear();
				point_cloud.height = xyz_height;
				point_cloud.width = xyz_width;
				point_cloud.points.resize(xyz_width*xyz_height);
				point_cloud.is_dense = false;
				for (int v=0; v<xyz_height; ++v)
				{
					uint16_t* depth_ptr = (uint16_t*)depth_image.ptr(v);
					for (int u=0; u<xyz_width; ++u, ++depth_ptr)
					{
						pcl::PointXYZ& point = point_cloud.at(u,v);
						openni::CoordinateConverter::convertDepthToWorld(depth_stream_, u, v, *depth_ptr, &point.x, &point.y, &point.z);
						point.x *= 0.001f;
						point.y *= 0.001f;
						point.z *= 0.001f;
					}
				}
			}
			else
				return false;
		}
		else
			return false;
	}

	// 2. XYZRGB point cloud and color_image
	if (process_xyzrgb || publish_color_image)
	{
		// acquire data from color camera
		openni::VideoFrameRef color_frame;
		if (color_stream_.isValid())
		{
			if (openni::STATUS_OK == color_stream_.readFrame(&color_frame))
			{
				// copy data
				const int xyzrgb_height = color_frame.getHeight();
				const int xyzrgb_width = color_frame.getWidth();
				color_image = cv::Mat(xyzrgb_height, xyzrgb_width, CV_8UC3, (void*)color_frame.getData());

				// write data into point cloud
				if (process_xyzrgb)
				{
					point_cloud_rgb.clear();
					point_cloud_rgb.height = xyzrgb_height;
					point_cloud_rgb.width = xyzrgb_width;
					point_cloud_rgb.points.resize(xyzrgb_width*xyzrgb_height);
					point_cloud_rgb.is_dense = false;
					for (int v=0; v<xyzrgb_height; ++v)
					{
						uint8_t* color_ptr = (uint8_t*)color_image.ptr(v);
						for (int u=0; u<xyzrgb_width; ++u)
						{
							pcl::PointXYZRGB& point = point_cloud_rgb.at(u,v);		// todo: replace .at with [] access for speedup
							const pcl::PointXYZ& point_depth = point_cloud.at(u,v);
							point.x = point_depth.x;
							point.y = point_depth.y;
							point.z = point_depth.z;
							point.r = *color_ptr; color_ptr++;
							point.g = *color_ptr; color_ptr++;
							point.b = *color_ptr; color_ptr++;
						}
					}
				}
			}
			else
				return false;
		}
		else
			return false;
	}

	return true;
}

//bool LipsCamera::activateSensor(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
//{
//	sensor_enabled_ = true;
//	res.success = true;
//	ROS_INFO("Sensor processing and publishing activated.");
//
//	return true;
//}
//
//bool LipsCamera::deactivateSensor(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
//{
//	sensor_enabled_ = false;
//	res.success = true;
//	ROS_INFO("Sensor processing and publishing deactivated.");
//
//	return true;
//}
//
//bool LipsCamera::activateSensorOnce(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
//{
//	sensor_enabled_once_ = true;
//	res.success = true;
//	ROS_INFO("Sensor processing and publishing activated for one frame.");
//
//	return true;
//}
