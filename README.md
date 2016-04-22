## Description
Program estimates normals in point cloud without them and creates new point cloud. Format of used point clouds is choosen by user - ply or pcd files.

---

+ Input: point cloud without normals - pcl::PointCloud<pcl::PointXYZ>
+ Output: point cloud with added estimated normals - pcl::PointCloud<pcl::PointXYZINormal>

---

This code is based on PCL tutorials available on: http://pointclouds.org/ 


## Usage
./point2normal cloud_without_normals(ply or pcd) name_of_new_cloud_with_normals(ply or pcd)

## Instalation
+ cd PCL-Cloud-Normal-Estimator
+ mkdir build && cd build
+ cmake ..
+ make 
