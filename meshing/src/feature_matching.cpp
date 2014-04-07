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
	icp.setInputSource(cloud_in_sub);
	icp.setInputTarget(cloud_out_sub);
	PointCloud<PointXYZRGBA> Final;
	icp.align(Final);

	writer.write("bal_sub_aligned.pcd", Final);

	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	return (0);
}
