#include <iostream>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h> // For pcl::getMinMax3D

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


int main(int argc, char* argv[])
{
  PointCloudT::Ptr cloud(new PointCloudT);
  PointCloudT::Ptr cloud_filtered(new PointCloudT);
  PointCloudT::Ptr cloud_Clean(new PointCloudT);


  if(argc < 2)
     {
         PCL_ERROR("The file.ply is missing : Provide one ply file.\n!");
         return(-1);
     }


  if (pcl::io::loadPLYFile(argv[1], *cloud) < 0)
  {
    PCL_ERROR("Error loading cloud %s.\n", argv[1]);
    return(-1);
  }

  // Vérification si le nuage est vide
    if (cloud->points.empty()) {
        std::cerr << "Point cloud is empty!" << std::endl;
        return -1;
    }
  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;


  // Vérification si des points contiennent des valeurs NaN
  for (const auto& point : cloud->points) {
      if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
          //std::cerr << "Point contains NaN value!" << std::endl;
          continue;
      }
      cloud_Clean->points.push_back(point);
  }
  std::cerr << "Deleting all Points containing NaN value!" << std::endl;

    // Si aucun point valide n'a été trouvé
    if (cloud_Clean->points.empty()) {
        std::cerr << "No valid points after filtering!" << std::endl;
        return -1;
    }
  // Calcul de la bounding box (boîte englobante)
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cloud_Clean, min_pt, max_pt);

    int downsampling_factor = 2;

    downsampling_factor = atoi(argv[2]);
  // Calcul des dimensions du nuage (largeur, hauteur, profondeur)
    float width = max_pt.x - min_pt.x;
    float height = max_pt.y - min_pt.y;
    float depth = max_pt.z - min_pt.z;
    // Affichage des dimensions
    std::cout << "Bounding box dimensions: " << std::endl;
    std::cout << "Width: " << width << std::endl;
    std::cout << "Height: " << height << std::endl;
    std::cout << "Depth: " << depth << std::endl;
  // Calcul de la taille du voxel en fonction de la taille du nuage
    float voxel_size = std::min(width, std::min(height, depth)) / (100 / downsampling_factor); // Taille de voxel proportionnelle à la plus petite dimension

  // Create the filtering object
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud_Clean);
  sor.setLeafSize(voxel_size, voxel_size, voxel_size);
  sor.filter(*cloud_filtered);


  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height

       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  pcl::visualization::PCLVisualizer viewer("Voxel Grid");
  // Create two vertically separated viewports
  int v1(0);
  int v2(1);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  // The color we will be using
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_h(cloud_Clean, (int)255*txt_gray_lvl,
                                          (int)255*txt_gray_lvl, (int)255*txt_gray_lvl);
  viewer.addPointCloud(cloud_Clean, cloud_color_h, "cloud_v1", v1);

  // downSampling point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_filtered, 20, 180, 20);
  viewer.addPointCloud(cloud_filtered, cloud_tr_color_h, "cloud_v2", v2);

  // Adding text descriptions in each viewport
  viewer.addText(" Original Points", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "Info_1", v1);
  viewer.addText(" Down Sampling Points", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "Info_2", v2);

  // Set background color
  viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  // Set camera position and orientation
  viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize(1280, 1024);  // Visualiser window size

  while(!viewer.wasStopped())
    {
      viewer.spinOnce();
    }

  return (0);

}
