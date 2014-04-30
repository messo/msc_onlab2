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

using namespace pcl;

int mainold() {
	PointCloud<PointXYZ>::Ptr cloud_in(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_out(new PointCloud<PointXYZ>);

	cloud_in->width = 4;
	cloud_in->height = 1;
	cloud_in->is_dense = false;
	cloud_in->points.resize(cloud_in->width * cloud_in->height);

	cloud_out->width = 4;
	cloud_out->height = 1;
	cloud_out->is_dense = false;
	cloud_out->points.resize(cloud_out->width * cloud_out->height);

	cloud_in->points[0].x = 2.55469;
	cloud_in->points[0].y = -0.539899;
	cloud_in->points[0].z = 12.6188;
	cloud_out->points[0].x = -1.57299;
	cloud_out->points[0].y = -0.0160064;
	cloud_out->points[0].z = 11.7474;

	cloud_in->points[1].x = 3.12895;
	cloud_in->points[1].y = 1.02275;
	cloud_in->points[1].z = 14.0861;
	cloud_out->points[1].x = -0.727981;
	cloud_out->points[1].y = 1.60138;
	cloud_out->points[1].z = 12.542;

	cloud_in->points[2].x = 3.63637;
	cloud_in->points[2].y = 1.12674;
	cloud_in->points[2].z = 14.4502;
	cloud_out->points[2].x = -0.2624;
	cloud_out->points[2].y = 1.71931;
	cloud_out->points[2].z = 12.7681;

	cloud_in->points[3].x = 2.74196;
	cloud_in->points[3].y = 0.537954;
	cloud_in->points[3].z = 13.0259;
	cloud_out->points[3].x = -1.34158;
	cloud_out->points[3].y = 1.06491;
	cloud_out->points[3].z = 11.6945;

	IterativeClosestPoint<PointXYZ, PointXYZ> icp;
	icp.setInputCloud(cloud_in);
	icp.setInputTarget(cloud_out);
	PointCloud<PointXYZ> Final;
	icp.align(Final);

	for (size_t i = 0; i < Final.points.size(); ++i) {
		std::cout << Final.points[i].x << " " << Final.points[i].y << " " << Final.points[i].z << std::endl;
	}

	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	return (0);

}

int main() {
	PointCloud<PointXYZRGBA>::Ptr cloud_in(new PointCloud<PointXYZRGBA>);
	PointCloud<PointXYZRGBA>::Ptr cloud_in_sub(new PointCloud<PointXYZRGBA>);
	PointCloud<PointXYZRGBA>::Ptr cloud_out(new PointCloud<PointXYZRGBA>);
	PointCloud<PointXYZRGBA>::Ptr cloud_out_sub(new PointCloud<PointXYZRGBA>);

	PCDReader reader;
	reader.read("bal.pcd", *cloud_in);
	reader.read("jobb.pcd", *cloud_out);

	// bal: (474, 126) -> (590, 284)
	// jobb: (146, 173) -> (278, 347)

	cloud_in_sub->width = 590-474;
	cloud_in_sub->height = 284-126;
	cloud_in_sub->points.resize(cloud_in_sub->width*cloud_in_sub->height);
	int i=0;
	for(int y=126; y<284; y++) {
		for(int x=474; x<590; x++) {
			PointXYZRGBA& p = cloud_in_sub->points[i];
			PointXYZRGBA& q = cloud_in->points[y*640 + x];
			p.x = q.x;
			p.y = q.y;
			p.z = q.z;
			p.rgba = q.rgba;
			++i;
		}
	}

	cloud_out_sub->width = 241-112;
	cloud_out_sub->height = 335-154;
	cloud_out_sub->points.resize(cloud_out_sub->width*cloud_out_sub->height);
	i=0;
	for(int y=154; y<335; y++) {
		for(int x=112; x<241; x++) {
			PointXYZRGBA& p = cloud_out_sub->points[i];
			PointXYZRGBA& q = cloud_out->points[y*640 + x];
			p.x = q.x;
			p.y = q.y;
			p.z = q.z;
			p.rgba = q.rgba;
			++i;
		}
	}

	PCDWriter writer;
	writer.write("bal_sub.pcd", *cloud_in_sub);
	writer.write("jobb_sub.pcd", *cloud_out_sub);

	IterativeClosestPoint<PointXYZRGBA, PointXYZRGBA> icp;
	icp.setInputCloud(cloud_in_sub);
	icp.setInputTarget(cloud_out_sub);
	PointCloud<PointXYZRGBA> Final;
	icp.align(Final);

	writer.write("bal_sub_aligned.pcd", Final);

	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	return (0);
}
