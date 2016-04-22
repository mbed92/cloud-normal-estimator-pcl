#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>

//types
typedef pcl::PointCloud<pcl::PointXYZ> XYZcloud;
typedef pcl::PointCloud<pcl::PointXYZINormal> XYZINcloud;
typedef pcl::PointCloud<pcl::Normal> Ncloud;

int main (int argc, char* argv[])
{
  //passed arguments
  if(argc != 3)
  {
    std::cout << "Usage: " << argv[0] << " cloud_without_normals name_of_new_cloud_with_normals" << std::endl;
    return 0;
  }

  //input cloud
  XYZcloud::Ptr cloudInput (new XYZcloud);	// cloud with only XYZ points
  Ncloud::Ptr cloudNormals (new Ncloud);	// cloud with only normals

  //output cloud
  XYZINcloud::Ptr cloudOutput (new XYZINcloud);	//cloud with XYZ points and normals
  
  //load PCD or PLY file
  std::string inputName = argv[1];
  std::string inputFileFormat = inputName.substr(inputName.size() - 3);
  if(inputFileFormat == "ply" || inputFileFormat == "PLY")
  {
    if( pcl::io::loadPLYFile(argv[1], *cloudInput) < 0 )
    {
      PCL_ERROR ("Error loading clouds %s\n", argv[1] );
      return (-1);
    }
  }
  else if(inputFileFormat == "pcd" || inputFileFormat == "PCD")
  {
    if( pcl::io::loadPCDFile(argv[1], *cloudInput) < 0 )
    {
      PCL_ERROR ("Error loading clouds %s\n", argv[1] );
      return (-1);
    }
  }

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloudInput);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloudNormals);
  
  // Print some info
  // 
  //std::cout << "Cloud Size:" << cloudInput->points.size() << " ----- " << std::endl
  //	    << "Width: " << cloudInput->width << " Height: " << cloudInput->height << std::endl;
  //
  
  //iterate through XYZ cloudInput and copy it to cloudOutput
  copyPointCloud(*cloudInput, *cloudOutput);

  //iterate through Normals cloudNormals and copy it to cloudOutput
  copyPointCloud(*cloudNormals, *cloudOutput);

  // print output cloud
  //for (size_t i = 0; i < cloudOutput->points.size(); ++i)
  //{
  //  std::cout << i << "    " 	<< "X: "  << cloudOutput->points[i].x  << " Y: "  <<  cloudOutput->points[i].y << " Z: "  <<  cloudOutput->points[i].z
  //	      << std::endl 	<< "NX: " << cloudOutput->points[i].normal_x << " NY: " << cloudOutput->points[i].normal_y << " NZ: " << cloudOutput->points[i].normal_z << std::endl;
  //}
  
  // save as PLY or PCD file
  std::string outputName = argv[2];
  std::string outputFileFormat = outputName.substr(outputName.size() - 3);
  if(outputFileFormat == "ply" || outputFileFormat == "PLY")
  {
    if( pcl::io::savePLYFile(argv[2], *cloudOutput) < 0 )
    {
      PCL_ERROR ("Error saving clouds %s\n", argv[2] );
      return (-1);
    }
  }
  else if(outputFileFormat == "pcd" || outputFileFormat == "PCD")
  {
    if( pcl::io::savePCDFile(argv[2], *cloudOutput) < 0 )
    {
      PCL_ERROR ("Error saving clouds %s\n", argv[2] );
      return (-1);
    }
  }
  return 0;
}
