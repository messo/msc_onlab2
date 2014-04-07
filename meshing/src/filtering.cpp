#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace pcl;

PointCloud<PointXYZ>::Ptr radiusOutlierRemoval(PointCloud<PointXYZ>::Ptr input, double radius, int count) {
	RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier_removal;

	radius_outlier_removal.setInputCloud(input);
	radius_outlier_removal.setRadiusSearch(radius);
	radius_outlier_removal.setMinNeighborsInRadius(count);
	radius_outlier_removal.setKeepOrganized(true);

	// do filtering
	PointCloud<PointXYZ>::Ptr cleaned(new PointCloud<PointXYZ>());
	radius_outlier_removal.filter(*cleaned);
	return cleaned;
}

PointCloud<PointXYZ>::Ptr statRemoval(PointCloud<PointXYZ>::Ptr input) {
	// Create the filtering object
	StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(input);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.setKeepOrganized(true);

	PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>());
	sor.filter(*cloud_filtered);
	return cloud_filtered;
}

int main(int argc, char** argv) {
	PointCloud<PointXYZ>::Ptr cloud1(new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr cloud2(new PointCloud<PointXYZ>());

	//std::cout << "searchRadius dilationIterations dilationVoxelSize";
	double radius = atof(argv[1]);
	int count = atoi(argv[2]);

	PCDReader reader;
	reader.read("balAffin.pcd", *cloud1);
	reader.read("jobbAffin.pcd", *cloud2);

	//std::vector<int> dummy;
	//removeNaNFromPointCloud(*cloud, *cloud, dummy);

	std::cout << "removing first..." << std::endl;
	PointCloud<PointXYZ>::Ptr removed1 = statRemoval(cloud1);
	std::cout << "removing second..." << std::endl;
	PointCloud<PointXYZ>::Ptr removed2 = statRemoval(cloud2);

	removed1 = radiusOutlierRemoval(removed1, radius, count);
	removed2 = radiusOutlierRemoval(removed2, radius, count);

	PCDWriter writer;
	writer.write<PointXYZ>("mine_removed_bal.pcd", *removed1, false);
	writer.write<PointXYZ>("mine_removed_jobb.pcd", *removed2, false);

	std::vector<int> dummy;
	removeNaNFromPointCloud(*removed1, *removed1, dummy);
	removeNaNFromPointCloud(*removed2, *removed2, dummy);
	writer.write<PointXYZ>("mine_removed_nonan_bal.pcd", *removed1, false);
	writer.write<PointXYZ>("mine_removed_nonan_jobb.pcd", *removed2, false);

//	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
//	tree->setInputCloud(cloud);
//
//	MovingLeastSquares<PointXYZ, PointXYZ> mls;
//	mls.setComputeNormals(false);
//	mls.setInputCloud(cloud);
//	mls.setPolynomialFit(true);
//	mls.setSearchMethod(tree);
//	mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::VOXEL_GRID_DILATION);
//	mls.setSearchRadius(searchRadius);
//	mls.setDilationIterations(dilationIterations);
//	mls.setDilationVoxelSize(dilationVoxelSize);
//
//	PointCloud<PointXYZ>::Ptr cloud_smoothed(new PointCloud<PointXYZ>());
//	mls.process(*cloud_smoothed);
//
//	writer.write<PointXYZ>("mine_smoothed.pcd", *cloud_smoothed, false);

	return (0);
}
