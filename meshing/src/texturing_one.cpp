#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

using namespace pcl;

void showCamera(texture_mapping::CameraVector cams, PointCloud<PointXYZ>::Ptr &cloud) {
    // visualization object
    visualization::PCLVisualizer visu("cameras");

    texture_mapping::Camera cam = cams[0];
    int i = 1;

    // add a visual for each camera at the correct pose
    double focal = cam.focal_length;
    double height = cam.height;
    double width = cam.width;

    // create a 5-point visual for each camera
    PointXYZ p1, p2, p3, p4, p5;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;
    double angleX = RAD2DEG(2.0 * atan(width / (2.0 * focal)));
    double angleY = RAD2DEG(2.0 * atan(height / (2.0 * focal)));
    double dist = 0.75;
    double minX, minY, maxX, maxY;
    maxX = dist * tan(atan(width / (2.0 * focal)));
    minX = -maxX;
    maxY = dist * tan(atan(height / (2.0 * focal)));
    minY = -maxY;
    p2.x = minX;
    p2.y = minY;
    p2.z = dist;
    p3.x = maxX;
    p3.y = minY;
    p3.z = dist;
    p4.x = maxX;
    p4.y = maxY;
    p4.z = dist;
    p5.x = minX;
    p5.y = maxY;
    p5.z = dist;
    p1 = transformPoint(p1, cam.pose);
    p2 = transformPoint(p2, cam.pose);
    p3 = transformPoint(p3, cam.pose);
    p4 = transformPoint(p4, cam.pose);
    p5 = transformPoint(p5, cam.pose);
    std::stringstream ss;
    ss << "Cam #" << i;
    visu.addText3D(ss.str(), p1, 0.1, 1.0, 1.0, 1.0, ss.str());

    ss.str("");
    ss << "camera_" << i << "line1";
    visu.addLine(p1, p2, ss.str());
    ss.str("");
    ss << "camera_" << i << "line2";
    visu.addLine(p1, p3, ss.str());
    ss.str("");
    ss << "camera_" << i << "line3";
    visu.addLine(p1, p4, ss.str());
    ss.str("");
    ss << "camera_" << i << "line4";
    visu.addLine(p1, p5, ss.str());
    ss.str("");
    ss << "camera_" << i << "line5";
    visu.addLine(p2, p5, ss.str());
    ss.str("");
    ss << "camera_" << i << "line6";
    visu.addLine(p5, p4, ss.str());
    ss.str("");
    ss << "camera_" << i << "line7";
    visu.addLine(p4, p3, ss.str());
    ss.str("");
    ss << "camera_" << i << "line8";
    visu.addLine(p3, p2, ss.str());

    // add a coordinate system
    // visu.addCoordinateSystem(1.0, "global");

    // add the mesh's cloud (colored on Z axis)
    visualization::PointCloudColorHandlerGenericField<PointXYZ> color_handler(cloud, "z");
    visu.addPointCloud(cloud, color_handler, "cloud");

    // reset camera
    visu.resetCamera();

    // wait for user input
    visu.spin();
}

int main(int argc, char** argv) {
    PolygonMeshPtr mesh(new PolygonMesh());
    io::loadPLYFile("mesh_voxel.ply", *mesh);

    // textúra mappelés
    TextureMesh tex_mesh;
    tex_mesh.cloud = mesh->cloud;
    tex_mesh.tex_polygons.push_back(mesh->polygons);

    std::cout << "\tInput mesh contains " << tex_mesh.tex_polygons[0].size() << " faces and " << mesh->cloud.height * mesh->cloud.width
	    << " vertices\n";

    std::cout << "\nLoading textures and camera poses...\n";
    texture_mapping::CameraVector my_cams;

    TextureMapping<PointXYZ>::Camera cam;

    cam.pose(0, 3) = 0;
    cam.pose(1, 3) = 0;
    cam.pose(2, 3) = 0;

    cam.pose(0, 0) = 1;
    cam.pose(0, 1) = 0;
    cam.pose(0, 2) = 0;

    cam.pose(1, 0) = 0;
    cam.pose(1, 1) = 1;
    cam.pose(1, 2) = 0;

    cam.pose(2, 0) = 0;
    cam.pose(2, 1) = 0;
    cam.pose(2, 2) = 1;

    cam.pose(3, 0) = 0.0;
    cam.pose(3, 1) = 0.0;
    cam.pose(3, 2) = 0.0;
    cam.pose(3, 3) = 1.0; //Scale

    cam.focal_length = 1160.0972941020677;
    cam.center_w = 268.11134910583496;
    cam.center_h = 260.91782188415527;
    cam.width = 640.0;
    cam.height = 480.0;
    cam.texture_file = "left01_bal.jpg";
    my_cams.push_back(cam);

    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
    fromPCLPointCloud2(mesh->cloud, *cloud);
    showCamera(my_cams, cloud);

    tex_mesh.tex_materials.resize(my_cams.size() + 1);
    for (int i = 0; i <= my_cams.size(); ++i) {
	TexMaterial mesh_material;
	mesh_material.tex_Ka.r = 0.2f;
	mesh_material.tex_Ka.g = 0.2f;
	mesh_material.tex_Ka.b = 0.2f;

	mesh_material.tex_Kd.r = 0.8f;
	mesh_material.tex_Kd.g = 0.8f;
	mesh_material.tex_Kd.b = 0.8f;

	mesh_material.tex_Ks.r = 1.0f;
	mesh_material.tex_Ks.g = 1.0f;
	mesh_material.tex_Ks.b = 1.0f;

	mesh_material.tex_d = 1.0f;
	mesh_material.tex_Ns = 75.0f;
	mesh_material.tex_illum = 2;

	std::stringstream tex_name;
	tex_name << "material_" << i;
	tex_name >> mesh_material.tex_name;

	if (i < my_cams.size())
	    mesh_material.tex_file = my_cams[i].texture_file;
	else
	    mesh_material.tex_file = "occluded.jpg";

	tex_mesh.tex_materials[i] = mesh_material;
    }

    std::cout << "\nSorting faces by cameras...\n";
    TextureMapping<PointXYZ> tm; // TextureMapping object that will perform the sort
    tm.textureMeshwithMultipleCameras(tex_mesh, my_cams);

    PCL_INFO("\nEstimating normals...\n");
    NormalEstimation<PointXYZ, Normal> n;
    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);

    // Concatenate XYZ and normal fields
    PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
    concatenateFields(*cloud, *normals, *cloud_with_normals);
    PCL_INFO("...Done.\n");

    toPCLPointCloud2(*cloud_with_normals, tex_mesh.cloud);

    io::saveOBJFile("mine.obj", tex_mesh);

    return (0);
}
