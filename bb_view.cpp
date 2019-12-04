// Basic lib
#include <vector>
#include <thread>

// PCL lib
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace std::literals::chrono_literals;

/* Things to do:
 * 
 * [] Include usage of text file for reading bounding box info
 * [] Custom build bounding box for info obtained from text file
 * [] If possible, aplha colouring bounding box area
 */

int main(int argc, char **argv)
{
  // File name, pcd and text only
  if (argc < 2 || argc > 3)
  {
    cout << "Usage: ./bb_view <pcd_file> <(optional) text_file>" << endl;
    return (0);
  }

  // Load pointcloud file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
    return (-1);

  // Create pcl viewer object and show the pointcloud
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "pointcloud");

  // If only pcd given, compute and show whole pcd bounding box
  // Else, create bounding boxes as per the text file specs
  if (argc == 2)
  {
    // Moment of Inertia - Feature extraction
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    // Variables definition
    // std::vector<float> moment_of_inertia; // unused variable
    // std::vector<float> eccentricity; // unused variable
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    // float major_value, middle_value, minor_value; // unused variable
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    // Variables setup
    // feature_extractor.getMomentOfInertia(moment_of_inertia); // unused variable
    // feature_extractor.getEccentricity(eccentricity); // unused variable
    // axis aligned bounding box
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    // oriented bounding box
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    // feature_extractor.getEigenValues(major_value, middle_value, minor_value); // unused variable
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter(mass_center);

    // Add the bounding box based on computed variables
    viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
    Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat(rotational_matrix_OBB);
    viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

    // Add the centre of the pcd as a coordinate axis
    pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
    pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
    pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
    pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
    viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
  }
  else
  {
    // Read all data from the text file
    

    // For each line, create a bounding box as described, iff the object is
    // Car/Pedestrian/Cyclist . . . marked with different colours
    
  }
  

  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(100ms);
  }

  return (0);
}
