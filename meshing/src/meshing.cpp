#include <iostream>
#include <string>
#include <sstream>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/conditional_removal.h>

using namespace pcl;

template<typename PointT>
typename PointCloud<PointT>::Ptr radiusOutlierRemoval(typename PointCloud<PointT>::Ptr input, double radius,
		int count) {
	RadiusOutlierRemoval<PointT> radius_outlier_removal;
	radius_outlier_removal.setInputCloud(input);
	radius_outlier_removal.setRadiusSearch(radius);
	radius_outlier_removal.setMinNeighborsInRadius(count);
	// radius_outlier_removal.setKeepOrganized(true);

	// do filtering
    typename PointCloud<PointT>::Ptr cleaned(new PointCloud<PointT>());
	radius_outlier_removal.filter(*cleaned);
	return cleaned;
}

PolygonMesh::Ptr greedyTriangulate(PointCloud<PointXYZ>::Ptr cloud, double searchRadius, int maximumNN) {
	NormalEstimation<PointXYZ, Normal> n;
	n.setInputCloud(cloud);

	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
	n.setSearchMethod(tree);

	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
    n.setRadiusSearch(0.4);
	n.compute(*normals);
	n.setViewPoint(0.035246f, 0.0173491f, 1.12414f);

	// Concatenate the XYZ and normal fields*
	PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
	concatenateFields(*cloud, *normals, *cloud_with_normals);

	PCDWriter w;
	w.write("output_normals.pcd", *cloud_with_normals);

	// Create search tree*
	search::KdTree<PointNormal>::Ptr tree2(new search::KdTree<PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	GreedyProjectionTriangulation<PointNormal> gp3;
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);

	// Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(searchRadius);

	// Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(maximumNN);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(true);
	gp3.setConsistentVertexOrdering(true);

	// Get result
    PolygonMesh::Ptr triangles(new PolygonMesh());
    gp3.reconstruct(*triangles);

    return triangles;
}

PointCloud<PointXYZ>::Ptr upsampling(PointCloud<PointXYZ>::Ptr cloud) {
	MovingLeastSquares<PointXYZ, PointXYZ> mls;
	mls.setInputCloud(cloud);
	mls.setSearchRadius(1.5);
	mls.setPolynomialFit(true);
	mls.setPolynomialOrder(3);
    mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(0.3); // 0.3
    mls.setUpsamplingStepSize(0.1); // 0.1

	PointCloud<PointXYZ>::Ptr output(new PointCloud<PointXYZ>());
	mls.process(*output);
	return output;
}

/*PointCloud<PointXYZ>::Ptr upsampling(PointCloud<PointXYZ>::Ptr cloud) {
    MovingLeastSquares<PointXYZ, PointXYZ> mls;
    mls.setInputCloud(cloud);
    mls.setSearchRadius(2.0);
    //mls.setPolynomialFit(true);
    //mls.setPolynomialOrder(3);
    mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::VOXEL_GRID_DILATION);
    mls.setDilationIterations(2); // 2
    mls.setDilationVoxelSize(0.1); // 0.1
    //mls.setUpsamplingRadius(0.3);
    //mls.setUpsamplingStepSize(0.1);

    PointCloud<PointXYZ>::Ptr output(new PointCloud<PointXYZ>());
    mls.process(*output);

    ConditionAnd<PointXYZ>::Ptr range_cond (new ConditionAnd<PointXYZ> ());
    range_cond->addComparison (FieldComparison<PointXYZ>::ConstPtr (new FieldComparison<PointXYZ> ("x", ComparisonOps::GT, -1000.0)));
    range_cond->addComparison (FieldComparison<PointXYZ>::ConstPtr (new FieldComparison<PointXYZ> ("x", ComparisonOps::LT, 1000.0)));
    // build the filter
    ConditionalRemoval<PointXYZ> condrem (range_cond);
    condrem.setInputCloud (output);
    condrem.setKeepOrganized(true);
    // apply filter
    PointCloud<PointXYZ>::Ptr output_filtered(new PointCloud<PointXYZ>());
    condrem.filter (*output_filtered);

    return output_filtered;
}*/

int main(int argc, char** argv) {
	PCDReader reader;
	PCDWriter writer;

	//double radius = atof(argv[1]);
	//int count = atoi(argv[2]);

	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
    reader.read("magic.pcd", *cloud);

    /*for(int i=1; i<15; i+=2) {
        OrganizedFastMesh<PointXYZ> orgMesh;
        PolygonMesh triangles;
        orgMesh.setTrianglePixelSize (i);
        orgMesh.setTriangulationType (OrganizedFastMesh<PointXYZ>::TRIANGLE_ADAPTIVE_CUT );
        orgMesh.setInputCloud(cloud);
        search::KdTree<PointXYZ>::Ptr tree2 (new search::KdTree<pcl::PointXYZ>);
        tree2->setInputCloud (cloud);
        orgMesh.setSearchMethod(tree2);
        orgMesh.reconstruct(triangles);

        std::ostringstream ss;
        ss << "org" << i << ".ply";
        io::savePLYFile(ss.str(), triangles);
    }*/

    std::vector<int> dummy;
    removeNaNFromPointCloud(*cloud, *cloud, dummy);
    /*for(int i=2; i<=2; i++) {
        PolygonMesh::Ptr triangles = greedyTriangulate(cloud, 1.5, i*50);

        std::ostringstream ss;
        ss << "greedy" << (i*50) << "_2.5.ply";
        io::savePLYFile(ss.str(), *triangles);
    }*/

//	std::vector<int> dummy;
//	removeNaNFromPointCloud(*cloud, *cloud, dummy);
//	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points ("
//			<< getFieldsList(*cloud) << "). \n";
//	writer.write("input_nonan.pcd", *cloud);


//	PointCloud<PointXYZ>::Ptr stat_removed(new PointCloud<PointXYZ>());
//	StatisticalOutlierRemoval<PointXYZ> sor;
//	sor.setInputCloud(cloud);
//	sor.setMeanK(200);
//	sor.setStddevMulThresh(1.0);
//	sor.filter(*stat_removed);
//	std::cerr << "PointCloud after stat removal: " << stat_removed->width * stat_removed->height << " data points ("
//			<< getFieldsList(*stat_removed) << "). \n";
//	writer.write("output_stat.pcd", *stat_removed);

//	PointCloud<PointXYZ>::Ptr removed_useless = radiusOutlierRemoval(cloud, radius1, count1);
//	std::cerr << "PointCloud after 1. radius removal: " << removed_useless->width * removed_useless->height
//			<< " data points (" << getFieldsList(*removed_useless) << "). \n";

	// Create the filtering object
    VoxelGrid<PointXYZ> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(0.15f, 0.15f, 1.0f);

    PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>());
    vox.filter(*cloud_filtered);

    /*for(int i=1; i<=10; i++) {
        VoxelGrid<PointXYZ> vox;
        vox.setInputCloud(cloud);
        vox.setLeafSize(0.05f*i, 0.05f*i, 1.0f);

        PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>());
        vox.filter(*cloud_filtered);

        PolygonMesh::Ptr triangles = greedyTriangulate(cloud_filtered, 1.5, 100);

        std::ostringstream ss;
        ss << "greedy_filtered_" << i << ".ply";
        io::savePLYFile(ss.str(), *triangles);
    }*/

    std::cerr << "PointCloud after filtering (voxel): " << cloud_filtered->width * cloud_filtered->height
			<< " data points (" << getFieldsList(*cloud_filtered) << ").\n";

    io::savePLYFile("cloud_filtered.ply", *cloud_filtered);

//	PointCloud<PointXYZ>::Ptr removed = radiusOutlierRemoval<PointXYZ>(cloud_filtered, radius, count);
//	std::cerr << "PointCloud after 2. radius removal: " << removed->width * removed->height << " data points ("
//			<< getFieldsList(*removed) << ").\n";
//	writer.write("output.pcd", *removed);

	PointCloud<PointXYZ>::Ptr upsampled = upsampling(cloud_filtered);
	std::cerr << "PointCloud after upsampling: " << upsampled->width * upsampled->height << " data points ("
			<< getFieldsList(*upsampled) << ").\n";
	writer.write("output_up.pcd", *upsampled);
    io::savePLYFile("output_up.ply", *upsampled);

	PointCloud<PointXYZ>::Ptr upsampled_filtered(new PointCloud<PointXYZ>());
	VoxelGrid<PointXYZ> sor2;
    sor2.setInputCloud(upsampled);
    sor2.setLeafSize(0.3f, 0.3f, 0.3f);
	sor2.filter(*upsampled_filtered);
	std::cerr << "PointCloud after upsampled filtering: " << upsampled_filtered->width * upsampled_filtered->height
			<< " data points (" << getFieldsList(*upsampled_filtered) << ").\n";
	writer.write("output_up_filtered.pcd", *upsampled_filtered);

    PolygonMesh::Ptr triangles = greedyTriangulate(upsampled_filtered, 1.5, 100);
    io::savePLYFile("mesh.ply", *triangles);

	return (0);
}
