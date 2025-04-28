#pragma once
#include <vector>
#include <string>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include "Util.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> pointCloud;

namespace regPI
{
	int Pointcloud2Pano(std::string &imgPath, std::string &posPath, std::string &ptsPath);
	bool readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud, Util::Offset &offset);
	bool saveLAS2(const std::string& filepath, const typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		const Util::Offset& offset);
	void world2camera(const Eigen::Matrix<double, 3, 3> &R, const Eigen::Matrix<double, 1, 3> &T, const pcl::PointXYZRGB &world_pts, cv::Point3d &camera_pts);
	void camera2pixel(cv::Point3d &camera_pts, cv::Point2d &uv, const int imgHeight, const int imgWidth);
}

