//For reference check http://pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php
#include <iostream>
#include <thread>
#include <vector>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/console/parse.h>

using namespace std::chrono_literals;

int
main (int argc, char** argv)
{

  if (argc < 3)
  {
    pcl::console::print_error ("Syntax is: %s"
                                "<pcd-input-file> <pcd-output-file>\n"
                                "-c <min cluster size>\n"
                                "-p <point color threshold>\n"
                                "-r <region color threshold>\n"
                                "-d <distance treshold>\n", argv[0]);
    return (1);
  }

  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (argv[1], *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }
  std::cout << "Cloud reading successful." << std::endl;

  double point_col = 7;
  if (pcl::console::find_switch (argc, argv, "-p"))
    pcl::console::parse (argc, argv, "-p", point_col);

  double reg_col = 16;
  if (pcl::console::find_switch (argc, argv, "-r"))
    pcl::console::parse (argc, argv, "-r", reg_col);

  double dist_tresh = 0.5;
  if (pcl::console::find_switch (argc, argv, "-d"))
    pcl::console::parse (argc, argv, "-d", dist_tresh);

  int min_cluster = 60;
  if (pcl::console::find_switch (argc, argv, "-c"))
    pcl::console::parse (argc, argv, "-c", min_cluster);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (dist_tresh);
  reg.setPointColorThreshold (point_col);
  reg.setRegionColorThreshold (reg_col);
  reg.setMinClusterSize (min_cluster);

  std::cout << "Extracting clusters." << std::endl;
  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;

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