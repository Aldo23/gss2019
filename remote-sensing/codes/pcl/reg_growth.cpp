//For reference check http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/console/parse.h>

int
main (int argc, char** argv)
{

  if (argc < 3)
  {
    pcl::console::print_error ("Syntax is: %s"
                                "<pcd-input-file> <pcd-output-file>\n"
                                "-min -max <cluster size limits>\n"
                                "-s <smoothness treshold>\n"
                                "-c <curvature treshold>\n", argv[0]);
    return (1);
  }


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[1], *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }
  std::cout << "Cloud reading successful." << std::endl;

  int cluster_min = 300;
  if (pcl::console::find_switch (argc, argv, "-min"))
    pcl::console::parse (argc, argv, "-min", cluster_min);

  int cluster_max = 100000000;
  if (pcl::console::find_switch (argc, argv, "-max"))
    pcl::console::parse (argc, argv, "-max", cluster_max);

  double smoothness_treshold = 1.1;
  if (pcl::console::find_switch (argc, argv, "-s"))
    pcl::console::parse (argc, argv, "-s", smoothness_treshold);

  double curvature_treshold = 0.4;
  if (pcl::console::find_switch (argc, argv, "-c"))
    pcl::console::parse (argc, argv, "-c", curvature_treshold);


  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);
  std::cout << "Normals estimated." << std::endl;

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (cluster_min);
  reg.setMaxClusterSize (cluster_max);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (smoothness_treshold / 180.0 * M_PI);
  reg.setCurvatureThreshold (curvature_treshold);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::io::savePCDFileASCII (argv[2], *colored_cloud);
  std::cout << "Segmented cloud saved as " << argv[2] << endl;
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ())
  {
  }

  return (0);
}
