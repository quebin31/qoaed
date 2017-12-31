#include <pcl/visualization/cloud_viewer.h>
#include <iostream>

#include "Tools.hpp"

int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ p;

  auto points = qoaed::tools::read_off(argv[1]);
  for (auto& t : points) {
    p.x = t.x;
    p.y = t.y;
    p.z = t.z;
    cloud->push_back(p);
  }

  pcl::visualization::CloudViewer viewer("Viewer");
  viewer.showCloud(cloud);
  while (!viewer.wasStopped()) {}
  return 0;
}
