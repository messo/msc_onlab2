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

void compute_surface_normals(PointCloud<pcl::PointXYZRGBA>::Ptr &points, float normal_radius, PointCloud<Normal>::Ptr &normals_out)
{
	NormalEstimation<PointXYZRGBA, Normal> norm_est;
	norm_est.setSearchMethod(search::KdTree<PointXYZRGBA>::Ptr(new search::KdTree<PointXYZRGBA>));
	norm_est.setRadiusSearch(normal_radius);
	norm_est.setInputCloud(points);
	norm_est.setSearchSurface(points);
	norm_est.compute(*normals_out);
}

void detect_keypoints(PointCloud<PointXYZRGBA>::Ptr &points, float min_scale,
					  int nr_octaves, int nr_scales_per_octave, float min_contrast,
					  PointCloud<PointWithScale>::Ptr &keypoints_out)
{
	SIFTKeypoint<PointXYZRGBA, PointWithScale> sift_detect;
	// Use a FLANN-based KdTree to perform neighborhood searches
	search::KdTree<PointXYZRGBA>::Ptr tree(new search::KdTree<PointXYZRGBA>());
	sift_detect.setSearchMethod(tree);
	// Set the detection parameters
	sift_detect.setScales(min_scale, nr_octaves, nr_scales_per_octave);
	sift_detect.setMinimumContrast(min_contrast);
	// Set the input
	sift_detect.setInputCloud(points);
	// Detect the keypoints and store them in "keypoints_out"
	sift_detect.compute(*keypoints_out);
}

void compute_PFH_features_at_keypoints(PointCloud<PointXYZRGBA>::Ptr &points,
									   PointCloud<Normal>::Ptr &normals,
									   PointCloud<PointWithScale>::Ptr &keypoints, float feature_radius,
									   PointCloud<PFHSignature125>::Ptr &descriptors_out)
{
	PFHEstimation<PointXYZRGBA, Normal, PFHSignature125> pfh_est;
	pfh_est.setSearchMethod(search::KdTree<PointXYZRGBA>::Ptr(new search::KdTree<PointXYZRGBA>));
	pfh_est.setRadiusSearch(feature_radius);

	PointCloud<PointXYZRGBA>::Ptr keypoints_xyzrgb(new PointCloud<PointXYZRGBA>);
	pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
	pfh_est.setSearchSurface(points);
	pfh_est.setInputNormals(normals);
	pfh_est.setInputCloud(keypoints_xyzrgb);
	pfh_est.compute(*descriptors_out);
}

const float bad_point = std::numeric_limits<float>::quiet_NaN();

int main()
{
	PointCloud<PointXYZRGBA>::Ptr cloud_in(new PointCloud<PointXYZRGBA>);
	PointCloud<Normal>::Ptr normals_in(new PointCloud<Normal>());
	PointCloud<PointWithScale>::Ptr keypoints_in(new PointCloud<PointWithScale>());
	PointCloud<PFHSignature125>::Ptr descriptors_in(new PointCloud<PFHSignature125>());

	PointCloud<PointXYZRGBA>::Ptr cloud_out(new PointCloud<PointXYZRGBA>);
	PointCloud<Normal>::Ptr normals_out(new PointCloud<Normal>());
	PointCloud<PointWithScale>::Ptr keypoints_out(new PointCloud<PointWithScale>());
	PointCloud<PFHSignature125>::Ptr descriptors_out(new PointCloud<PFHSignature125>());

	std::cout << "Reading files..." << std::endl;

	PCDReader reader;
	reader.read("bal1.pcd", *cloud_in);
	reader.read("jobb1.pcd", *cloud_out);

	std::cout << "Computing normals..." << std::endl;
	compute_surface_normals(cloud_in, 0.2f, normals_in);
	std::cout << "Detecting keypoints..." << std::endl;
	detect_keypoints(cloud_in, 0.01f, 4, 5, 1.0f, keypoints_in);
	std::cout << "Features..." << std::endl;
	compute_PFH_features_at_keypoints(cloud_in, normals_in, keypoints_in, 0.1f, descriptors_in);

	std::cout << "Next..." << std::endl;
	std::cout.flush();

	std::cout << "Computing normals..." << std::endl;
	compute_surface_normals(cloud_out, 0.2f, normals_out);
	std::cout << "Detecting keypoints..." << std::endl;
	detect_keypoints(cloud_out, 0.01f, 4, 5, 1.0f, keypoints_out);
	std::cout << "Features..." << std::endl;
	compute_PFH_features_at_keypoints(cloud_out, normals_out, keypoints_out, 0.1f, descriptors_out);

	std::cout << descriptors_in->width * descriptors_in->height << std::endl;
	std::cout << descriptors_out->width * descriptors_out->height << std::endl;
	std::cout.flush();

	registration::CorrespondenceEstimation<PFHSignature125, PFHSignature125> est;
	est.setInputCloud(descriptors_in);
	est.setInputTarget(descriptors_out);
	Correspondences correspondences;
	est.determineCorrespondences(correspondences);

	std::cout << "CorrespondenceRejector..." << std::endl;
	std::cout.flush();

	registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBA> sac;
	sac.setInputCloud(cloud_in);
	sac.setTargetCloud(cloud_out);
	sac.setInlierThreshold(0.2f);
	sac.setMaxIterations(10);
	CorrespondencesConstPtr cptr(&correspondences);
	sac.setInputCorrespondences(cptr);
	Correspondences inliers;
	sac.getCorrespondences(inliers);
	Eigen::Matrix4f transformation = sac.getBestTransformation();

	std::cout << "Done..." << inliers.size() << std::endl;
	std::cout << transformation;
	std::cout.flush();

	visualization::PCLVisualizer visu("cameras");

//	bool alter = true;
//	for (size_t i = 0; i < inliers.size(); i++) {
//		PointWithScale& p_src = keypoints_in->points.at(inliers[i].index_match);
//		PointWithScale& p_tgt = keypoints_out->points.at(inliers[i].index_query);

//		std::stringstream ss("line");
//		ss << i;
//		std::stringstream sss("spheresource");
//		sss << i;
//		std::stringstream ssss("spheretarget");
//		ssss << i;

//		if (alter) {
//			//this is for red lines and spheres
//			visu.addSphere<pcl::PointWithScale>(p_src, 0.5, 255, 0, 0, sss.str());
//			visu.addSphere<pcl::PointWithScale>(p_tgt, 0.5, 255, 0, 0, ssss.str());
//			visu.addLine<pcl::PointWithScale>(p_src, p_tgt, 0, 0, 255, ss.str());
//		} else {
//			//this is for yellow ones
//			visu.addSphere<pcl::PointWithScale>(p_src, 0.5, 255, 255, 0, sss.str());
//			visu.addSphere<pcl::PointWithScale>(p_tgt, 0.5, 255, 255, 0, ssss.str());
//			visu.addLine<pcl::PointWithScale>(p_src, p_tgt, 220, 24, 225, ss.str());
//		}
//		alter = !alter;
//	}

	// visu.addPointCloudNormals<PointXYZRGBA, Normal>(cloud_in, normals_out, 100, 0.05f);
	visu.addPointCloud(cloud_in, "cloud2");
	visu.addPointCloud(cloud_out, "cloud3");

	PointCloud<PointXYZRGBA>::Ptr transformed(new PointCloud<PointXYZRGBA>());
	pcl::transformPointCloud(*cloud_in, *transformed, transformation);
	visu.addPointCloud(transformed, "cloud4");

	// reset camera
	visu.resetCamera();

	// wait for user input
	visu.spin();

	return (0);
}
