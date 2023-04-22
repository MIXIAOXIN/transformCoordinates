#define _USE_MATH_DEFINES //需要放在math前,之后才可以使用M_PI等match定义参数
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <LASlib/lasreader.hpp>
#include <LASlib/laswriter.hpp>
#include "Pts2pano.h"
#include "Util.h"
#include "poseRead.h"
#include "BoundingBox.h"
#include "BoundingBoxApply.hpp"

typedef pcl::PointCloud<pcl::PointXYZRGB> pointCloud;

bool regPI::readLasFile(const std::string &fileName, pointCloud::Ptr &pointCloud, Util::Offset &offset)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}

	std::ifstream infilePoint;
	infilePoint.open(fileName, std::ios::in | std::ios::binary);
	if (infilePoint.bad())
	{
		std::cout << "FileName Error!" << std::endl;
	}
	infilePoint.close();

	LASreadOpener lasReadOpener;
	lasReadOpener.set_file_name(fileName.c_str());
	LASreader* lasReader = lasReadOpener.open(false);

	offset.x = lasReader->header.x_offset;
	offset.y = lasReader->header.y_offset;
	offset.z = lasReader->header.z_offset;

	int pts_idx = 0;
	while (lasReader->read_point())
	{
		LASpoint& ptReader = lasReader->point;
		pcl::PointXYZRGB pt;
		pt.x = ptReader.get_x() - offset.x;
		pt.y = ptReader.get_y() - offset.y;
		pt.z = ptReader.get_z() - offset.z;
		pt.r = 0;
		pt.g = 0;
		pt.b = 0;
		pointCloud->push_back(pt);
	}
	return 1;
}

bool regPI::saveLAS2(const std::string& filepath, const typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
	const Util::Offset& offset)
{
	if (cloud == nullptr)
	{
		std::cout << "Pointer cloud is nullptr!" << std::endl;
		return false;
	}
	Util::BoundingBox<float> boundbox = Util::getBoundBox<pcl::PointXYZRGB, float>(cloud);
	if (cloud->empty())
	{
		std::cout << "Point cloud is empty!" << std::endl;
		return false;
	}
	// LASlib write las file
	LASwriteOpener laswriteopener;
	laswriteopener.set_file_name(filepath.c_str());
	
	LASheader *lasheader = new LASheader();
	
	// Normal header setting
	strncpy(lasheader->system_identifier, "Group.Yang", 11);
	lasheader->system_identifier[10] = '\0';
	strncpy(lasheader->generating_software, "2.0", 4);
	lasheader->generating_software[3] = '\0';
	lasheader->version_major = 1;
	lasheader->version_minor = 2;
	lasheader->set_bounding_box(boundbox.m_bbMin.x() + offset.x, boundbox.m_bbMin.y() + offset.y, boundbox.m_bbMin.z() + offset.z,
		boundbox.m_bbMax.x() + offset.x, boundbox.m_bbMax.y() + offset.y, boundbox.m_bbMax.z() + offset.z,
		false, false);
	lasheader->header_size = 227;
	lasheader->offset_to_point_data = 227;
	// XYZRGB
	lasheader->point_data_format = 2;
	lasheader->point_data_record_length = 26;
	// Geometry info
	lasheader->number_of_point_records = (U32)cloud->size();
	lasheader->x_scale_factor = 0.0001;
	lasheader->y_scale_factor = 0.0001;
	lasheader->z_scale_factor = 0.0001;
	lasheader->x_offset = offset.x;
	lasheader->y_offset = offset.y;
	lasheader->z_offset = offset.z;
	LASwriter* laswriter = laswriteopener.open(lasheader);
	if (laswriter == 0)
	{
		std::cout << "Could not open laswriter" << std::endl;
		return false;
	}
	LASpoint *point = new LASpoint();
	point->init(lasheader, lasheader->point_data_format, lasheader->point_data_record_length, 0);
	// write point clouds to las file with rgb
	for (int i = 0; i < cloud->size(); ++i)
	{
		double x = static_cast<double>(cloud->points[i].x) + offset.x;
		double y = static_cast<double>(cloud->points[i].y) + offset.y;
		double z = static_cast<double>(cloud->points[i].z) + offset.z;
		// here the coordinate just have offset with out scale, so should use set_x instead of set_X
		point->set_x(x);
		point->set_y(y);
		point->set_z(z);
		point->set_intensity(10);
		point->rgb[0] = U16_QUANTIZE(cloud->points[i].r);
		point->rgb[1] = U16_QUANTIZE(cloud->points[i].g);
		point->rgb[2] = U16_QUANTIZE(cloud->points[i].b);
		// write the modified point
		laswriter->write_point(point);
		laswriter->update_inventory(point);
	}
	laswriter->update_header(lasheader, TRUE);
	I64 total_bytes = laswriter->close();
	if (laswriter == 0)
	{
		std::cout << "ERROR: could not open laswriter" << std::endl;
	}
	delete laswriter;
	std::cout << "Save point file: " + filepath << std::endl;
	return true;
}

void regPI::world2camera(const Eigen::Matrix<double, 3, 3> &R, const Eigen::Matrix<double, 1, 3> &T, const pcl::PointXYZRGB &world_pts, cv::Point3d &camera_pts)
{
	camera_pts.x = R(0, 0) * (world_pts.x - T(0, 0)) + R(0, 1) * (world_pts.y - T(0, 1)) + R(0, 2) * (world_pts.z - T(0, 2));
	camera_pts.y = R(1, 0) * (world_pts.x - T(0, 0)) + R(1, 1) * (world_pts.y - T(0, 1)) + R(1, 2) * (world_pts.z - T(0, 2));
	camera_pts.z = R(2, 0) * (world_pts.x - T(0, 0)) + R(2, 1) * (world_pts.y - T(0, 1)) + R(2, 2) * (world_pts.z - T(0, 2));
}

void regPI::camera2pixel(cv::Point3d &camera_pts, cv::Point2d &uv, const int imgHeight, const int imgWidth)
{
	/*double theta = 180 + atan2(camera_pts.y, camera_pts.x) * 180 / M_PI;
	double phi = atan2(sqrt(camera_pts.x*camera_pts.x + camera_pts.y*camera_pts.y), camera_pts.z) * 180 / M_PI;
	uv.x = int(imgWidth - (theta / 360.0)*imgWidth);
	uv.y = int(imgHeight * phi / 180.0);*/

	//右手坐标系，phi定义为与Z轴正向的夹角，theta定义为与y轴正向的夹角
	double theta = atan2(camera_pts.x, camera_pts.y) * 180 / M_PI; 
	if (theta == M_PI)
	{
		theta = -M_PI;
	}
	double phi = atan2(sqrt(camera_pts.x*camera_pts.x + camera_pts.y*camera_pts.y), camera_pts.z) * 180 / M_PI;
	uv.x = int((180 + theta)*imgWidth / 360.0);
	uv.y = int(imgHeight * phi / 180.0);
}

int regPI::Pointcloud2Pano(std::string &imgPath, std::string &posPath, std::string &ptsPath)
{
	std::string imgExt = ".jpg";
	std::vector<std::string> imgFile;
	Util::get_files(imgPath, imgExt, imgFile);

	std::string ptsExt = ".las";
	std::vector<std::string> ptsFile;
	Util::get_files(imgPath, ptsExt, ptsFile);

	std::vector<regPI::myPose> readPose;
	std::string posExt;
	std::string rightPosExt = ".txt";
	posExt = Util::getExtend(posPath);
	if (posExt == rightPosExt)
	{
		regPI::loadPose(posPath, readPose);
	}
	else
	{
		std::cout << rightPosExt << ", " << posExt << std::endl;
		std::cout << "文件格式不支持" << std::endl;
	}
	
	for (int img_i = 0; img_i < imgFile.size(); img_i++)
	{
		std::cout << "已完成: " << img_i << "/" << imgFile.size() << std::endl;
		//读取全景图像
		cv::Mat image = cv::imread(imgFile[img_i], 1);
		if (image.empty())
		{
			std::cout << "Cannot load image." << std::endl;
			return 0;
		}
		int imgHeight = image.rows, imgWidth = image.cols;

		int imgTime;
		Eigen::Matrix<double, 1, 3> rotDegree; //存放角度信息
		Eigen::Matrix<double, 1, 3> exterT; //存放偏移信息
		std::string imgName = Util::getNameWithoutExtend(imgFile[img_i]);
		for (int pos_i = 0; pos_i < readPose.size(); pos_i++)
		{
			std::string tempName = readPose[pos_i].imgName;
			//std::cout << tempName << ", " << imgName << std::endl;
			if (tempName == imgName)
			{
				imgTime = (int)readPose[pos_i].time;
				rotDegree[0] = readPose[pos_i].roll;
				rotDegree[1] = readPose[pos_i].pitch;
				rotDegree[2] = readPose[pos_i].yaw;
				exterT[0] = readPose[pos_i].Tx;
				exterT[1] = readPose[pos_i].Ty;
				exterT[2] = readPose[pos_i].Tz;
				break;
			}
		}
		Eigen::Matrix<double, 3, 3> rotR;
		rotR = Eigen::AngleAxisd(-rotDegree[0] * M_PI/180, Eigen::Vector3d::UnitX())
			   * Eigen::AngleAxisd(rotDegree[1] * M_PI / 180, Eigen::Vector3d::UnitY())
			   * Eigen::AngleAxisd((rotDegree[2]-90) * M_PI / 180, Eigen::Vector3d::UnitZ());

		//读取点云
		int imgTime_last_number = imgTime % 10;
		std::string pointCloudFile = ptsPath + "\\" + std::to_string(imgTime - imgTime_last_number) 
			                         + "_" + std::to_string(imgTime - imgTime_last_number + 10) + ".las";
		Util::Offset offset;
		pointCloud::Ptr world_pts = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();//指针开辟内存空间
		regPI::readLasFile(pointCloudFile, world_pts, offset);

		std::vector<cv::Point2d> uv_pts;
		for (int p = 0; p < world_pts->size(); p++)
		{
			cv::Point3d camera_pts;
			pcl::PointXYZRGB temp_pt = world_pts->points[p];
			temp_pt.x += offset.x;
			temp_pt.y += offset.y;
			temp_pt.z += offset.z;
			regPI::world2camera(rotR, exterT, temp_pt, camera_pts);

			cv::Point2d uv;
			regPI::camera2pixel(camera_pts, uv, imgHeight, imgWidth);

			world_pts->points[p].b = image.at<cv::Vec3b>(uv.y, uv.x)[0];
			world_pts->points[p].g = image.at<cv::Vec3b>(uv.y, uv.x)[1];
			world_pts->points[p].r = image.at<cv::Vec3b>(uv.y, uv.x)[2];
			uv_pts.push_back(uv);
		}

		std::string panoPath = Util::getParent(imgFile[img_i]);
		std::string imgSavePath = panoPath + "\\" + imgName + "_proj.jpg";

		for (int p_i = 0; p_i < uv_pts.size(); p_i++)
		{
			cv::Point2d temp_uv = uv_pts[p_i];
			cv::circle(image, cv::Point(temp_uv.x, temp_uv.y), 1, cv::Scalar(0, 0, 255), 1); //circle需opencv_imgproc.lib
		}
		cv::imwrite(imgSavePath, image);
		std::cout << "\nSave project pano file: " + imgSavePath << std::endl;

		std::string ptsPath = Util::getParent(pointCloudFile);
		std::string ptsName = Util::getNameWithoutExtend(pointCloudFile);
		std::string ptsSavePath = ptsPath + "\\" + ptsName + "_rgb.las";

		regPI::saveLAS2(ptsSavePath, world_pts, offset);
	}	
	return 1;
}
