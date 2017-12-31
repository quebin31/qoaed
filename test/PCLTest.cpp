#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_picking_event.h>
#include <iostream>

#include "Tools.hpp"
#include "PointOctree.hpp"


void foo(const pcl::visualization::PointPickingEvent& e, void* point) {
  float x, y, z;
  e.getPoint(x,y,z);
  qoaed::Point3D<double> *p = (qoaed::Point3D<double>*) (point);
  p->x = (double) x;
  p->y = (double) y;
  p->z = (double) z;
  std::cout << p->x << ' ' << p->y << ' ' << p->z << std::endl;
}

int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  //qoaed::PointOctree<pcl::PointXYZRGB, double> octree;

  pcl::PointXYZRGB p;
  auto points = qoaed::tools::read_off<double>(argv[1]);
  for (qoaed::Point3D<double>& t : points) {
    p.x = t.x;
    p.y = t.y;
    p.z = t.z;
    p.r = 255;
    p.g = 255;
    p.b = 255;
    cloud->push_back(p);
    //octree.insert(t, p);
  }

  qoaed::Point3D<double> pqoaed;
  pcl::visualization::CloudViewer viewer("Viewer");
  viewer.registerPointPickingCallback(foo, &pqoaed);
  viewer.showCloud(cloud);
  viewer.showCloud(cloud);
  while (!viewer.wasStopped()) {}
  return 0;
}
