#include <pcl/visualization/cloud_viewer.h>
#include <iostream>

int main() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ p;
  p.x = 3;
  p.y = 3;
  p.z = 1;
  cloud->push_back(p);
  p.x = 1;
  p.y = 1;
  p.z = 1;
  cloud->push_back(p);
  p.x = 4;
  p.y = 7;
  p.z = 2;
  cloud->push_back(p);
  p.x = 1;
  p.y = 5;
  p.z = 4;
  cloud->push_back(p);
  p.x = 5;
  p.y = 5;
  p.z = 7;
  cloud->push_back(p);

  pcl::visualization::CloudViewer viewer("Viewer");
  viewer.showCloud(cloud);
  while (!viewer.wasStopped()) {}
  return 0;
}
