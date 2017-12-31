#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_picking_event.h>
#include <iostream>

#include "Point.hpp"
#include "Tools.hpp"


void foo(const pcl::visualization::PointPickingEvent& e, void* something) {
  float x, y, z;
  e.getPoint(x,y,z);
  qoaed::Point3D<float> *p = (qoaed::Point3D<float>*) (something);
  p->x = x;
  p->y = y;
  p->z = z;
  std::cout << p->x << ' ' << p->y << ' ' << p->z << std::endl;
}

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

  qoaed::Point3D<float> pqoaed;
  pcl::visualization::CloudViewer viewer("Viewer");
  viewer.registerPointPickingCallback(foo, &p);
  viewer.showCloud(cloud);
  while (!viewer.wasStopped()) {}
  return 0;
}
