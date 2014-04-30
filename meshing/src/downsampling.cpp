/*
 * merge.cpp
 *
 *  Created on: 2014.04.04.
 *      Author: Balint
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

using namespace pcl;

const float bad_point = std::numeric_limits<float>::quiet_NaN();

int main()
{
	PointCloud<PointXYZRGBA>::Ptr cloud_in(new PointCloud<PointXYZRGBA>);
	PointCloud<PointXYZRGBA>::Ptr cloud_out(new PointCloud<PointXYZRGBA>);

	PCDReader reader;
	reader.read("bal.pcd", *cloud_in);
	reader.read("jobb.pcd", *cloud_out);

	// original bal
	//for (size_t i = 0; i < cloud_in->points.size(); i++) {
//		if (cloud_in->points[i].z > 15.0f || cloud_in->points[i].z < 10.0f || cloud_in->points[i].x < -2.0f) {
//			cloud_in->points[i].x = cloud_in->points[i].y = cloud_in->points[i].z = bad_point;
//		}
//	}

	for (size_t i = 0; i < cloud_in->points.size(); i++) {
		if (cloud_in->points[i].z > 12.0f || cloud_in->points[i].z < 9.5f || cloud_in->points[i].x < -2.0f) {
			cloud_in->points[i].x = cloud_in->points[i].y = cloud_in->points[i].z = bad_point;
		}
	}

	for (size_t i = 0; i < cloud_out->points.size(); i++) {
		if (cloud_out->points[i].z > 15.0f || cloud_out->points[i].z < 10.0f || cloud_out->points[i].x < -2.0f) {
			cloud_out->points[i].x = cloud_out->points[i].y = cloud_out->points[i].z = bad_point;
		} else {
			cloud_out->points[i].x += 10.0f;
		}
	}

	std::vector<int> dummy;
	removeNaNFromPointCloud(*cloud_in, *cloud_in, dummy);
	removeNaNFromPointCloud(*cloud_out, *cloud_out, dummy);

	visualization::PCLVisualizer visu("cameras");

	PCDWriter writer;
	writer.write("bal1.pcd", *cloud_in);
	writer.write("jobb1.pcd", *cloud_out);

	visu.addPointCloud(cloud_in, "cloud2");
	visu.addPointCloud(cloud_out, "cloud3");

	visu.resetCamera();
	visu.spin();

	return (0);
}
