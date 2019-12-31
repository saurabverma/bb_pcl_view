
/* Aim: Create a bounding box (about a pcl data) as dictated by the labels
 * 
 * Author: Dr. Saurab Verma
 */

// Basic lib
#include <cmath>
#include <vector>
#include <thread>
#include <fstream>

// PCL lib
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;
using namespace std::literals::chrono_literals;
using namespace Eigen;

vector<string> string_split(string s, string delimiter)
{
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  string token;
  vector<string> res;

  while ((pos_end = s.find(delimiter, pos_start)) != string::npos)
  {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    res.push_back(token);
  }

  res.push_back(s.substr(pos_start));
  return res;
}

int main(int argc, char **argv)
{
  //Help message
  auto help_msg = "Usage: ./bb_view pcd_file <option(s)>\n\tpcd_file: path to pcd file to show\n\t-h: show this help message\n\t-l: path to text file containing label information (in kitti format)\n\t-t: threshold limit on scores\n\t-k: use Kitti specificed camera to lidar transformations";

  // Setup input arguments
  float threshold = -1000.0;
  string label_filename = "";
  bool kitti = false;
  for (auto i = 1; i < argc; ++i)
  {
    string arg = argv[i];
    if (arg == "-h")
    {
      cout << help_msg << endl;
      return 0;
    }
    else if (arg == "-l")
    {
      label_filename = argv[i + 1];
    }
    else if (arg == "-t")
    {
      // Set threshold
      threshold = stod(argv[i + 1]);
      // DEBUG: Print out info
      cout << "Threshold confidence value: " << threshold << endl;
    }
    else if (arg == "-k")
    {
      kitti = true;
    }
  }

  // Load pointcloud file
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
  if (io::loadPCDFile(argv[1], *cloud) == -1)
  {
    cout << "Unable to open pcd file: " << argv[1] << endl
         << help_msg << endl;
    return 1; // terminate with error
  }

  // Create pcl viewer object and show the pointcloud
  visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(5.0, "lidar");
  viewer->initCameraParameters();
  viewer->addPointCloud<PointXYZ>(cloud, "pointcloud");

  // If only pcd given, compute and show whole pcd bounding box
  // Else, create bounding boxes as per the text file specs
  if (label_filename == "")
  {
    // Moment of Inertia - Feature extraction
    MomentOfInertiaEstimation<PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    // Variables definition
    PointXYZ min_point_AABB;
    PointXYZ max_point_AABB;
    PointXYZ min_point_OBB;
    PointXYZ max_point_OBB;
    PointXYZ position_OBB;
    Matrix3f rotational_matrix_OBB;
    Vector3f major_vector, middle_vector, minor_vector;
    Vector3f mass_center;

    // Variables setup
    // axis aligned bounding box
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    // oriented bounding box
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter(mass_center);

    // Add the bounding box based on computed variables
    viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
    viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_REPRESENTATION, visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
    Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
    Quaternionf orientation(rotational_matrix_OBB);
    viewer->addCube(position, orientation, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_REPRESENTATION, visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

    // Add the centre of the pcd as a coordinate axis
    PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
    PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
    PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
    PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
    viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
  }
  else // FIXME:
  {
    // Calibration matrix manual setup
    Matrix4f T_lidar_wrt_camera_frame;
    T_lidar_wrt_camera_frame.setIdentity();
    Quaternion<float> q = AngleAxisf(0.0, Vector3f::UnitX()) * AngleAxisf(0.0, Vector3f::UnitY()) * AngleAxisf(0.0, Vector3f::UnitZ());
    if (kitti)
    {
      // Translation
      T_lidar_wrt_camera_frame(1, 3) = -0.08;
      T_lidar_wrt_camera_frame(2, 3) = -0.27;
      // Rotation
      q = AngleAxisf(M_PI / 2, Vector3f::UnitX()) * AngleAxisf(0.0, Vector3f::UnitY()) * AngleAxisf(M_PI / 2, Vector3f::UnitZ());
    }
    q.normalize();
    T_lidar_wrt_camera_frame.block(0, 0, 3, 3) = q.matrix();
    // DEBUG: Print out info
    if (kitti)
      cout << "User-input transformation of Camera with respect to Lidar frame: " << endl
           << T_lidar_wrt_camera_frame << endl;

    // Invert tf, lidar wrt camera
    Matrix4f T_camera_wrt_lidar_frame;
    T_camera_wrt_lidar_frame.setIdentity();
    // Translation
    T_camera_wrt_lidar_frame.block(0, 3, 3, 1) = -q.matrix().transpose() * T_lidar_wrt_camera_frame.block(0, 3, 3, 1);
    // Rotation
    T_camera_wrt_lidar_frame.block(0, 0, 3, 3) = q.matrix().transpose();
    // Print out info
    cout << "Computed transformation of Lidar with respect to Camera frame: " << endl
         << T_camera_wrt_lidar_frame << endl;

    // DEBUG: Visualization
    Affine3f T_camera_wrt_lidar_frame_(T_camera_wrt_lidar_frame);
    viewer->addCoordinateSystem(5.0, T_camera_wrt_lidar_frame_, "camera");

    // Open text file
    string line;
    ifstream TextFile;
    TextFile.open(label_filename);
    if (!TextFile)
    {
      cout << "Unable to open text file" << endl;
      return 1; // terminate with error
    }

    // For each line in the file, create a bounding box as described, iff the
    // object is Car/Pedestrian/Cyclist . . . marked with different colours
    int item_count = 0;
    while (getline(TextFile, line))
    {
      // Check if true object
      int colour = 0;
      if (line.find("Car ") != string::npos)
      {
        colour = 1;
      }
      else if (line.find("Pedestrian ") != string::npos || line.find("Truck ") != string::npos || line.find("Van ") != string::npos)
      // else if (line.find("Pedestrian ") != string::npos || line.find("Truck ") != string::npos)
      {
        colour = 2;
      }
      else if (line.find("Cyclist ") != string::npos)
      {
        colour = 3;
      }

      // Add bounding box with required colour scheme
      if (colour != 0)
      {
        // Extract data from file
        vector<string> data = string_split(line, " ");
        // for (auto i : data) cout << i << endl; // DEBUG:

        // data = cfg.CLASSES, -1, -1, alpha, img_boxes[k, 0], img_boxes[k, 1], img_boxes[k, 2], img_boxes[k, 3], bbox3d[k, 3], bbox3d[k, 4], bbox3d[k, 5], bbox3d[k, 0], bbox3d[k, 1], bbox3d[k, 2], bbox3d[k, 6], scores[k]

        // k = object number inside each frame
        // height, width, length = bbox3d[k, 3], bbox3d[k, 4], bbox3d[k, 5] = data[8:10]
        // x, y, z (in camera coordinates) = bbox3d[k, 0], bbox3d[k, 1], bbox3d[k, 2] = data[11:13]
        // Rotation ry around Y-axis in camera coordinates = bbox3d[k, 6] = data[14]

        // Variables setup
        item_count++;
        float height = stod(data[8]);
        float width = stod(data[9]);
        float length = stod(data[10]);
        float pos_x = stod(data[11]);
        float pos_y = stod(data[12]);
        float pos_z = stod(data[13]);
        float ry = stod(data[14]);
        float confidence;
        if (data.size() == 16)
        {
          confidence = stod(data[15]);
        }
        else
        {
          confidence = threshold + 1.0;
        }

        if (confidence > threshold)
        {
          // Setup camera to bounding box transformation
          Matrix4f T_BB_wrt_camera_frame;
          T_BB_wrt_camera_frame.setIdentity();
          // Translation
          T_BB_wrt_camera_frame(0, 3) = pos_x;
          T_BB_wrt_camera_frame(1, 3) = pos_y;
          T_BB_wrt_camera_frame(2, 3) = pos_z;
          // Rotation
          Quaternion<float> q;
          if (kitti)
          {
            q = AngleAxisf(0.0, Vector3f::UnitX()) * AngleAxisf(ry, Vector3f::UnitY()) * AngleAxisf(0.0, Vector3f::UnitZ());
          }
          else
          {
            q = AngleAxisf(0.0, Vector3f::UnitX()) * AngleAxisf(0.0, Vector3f::UnitY()) * AngleAxisf(ry, Vector3f::UnitZ());
          }
          q.normalize();
          T_BB_wrt_camera_frame.block(0, 0, 3, 3) = q.matrix();

          // // DEBUG: Print out info
          // cout << "Computed transformation of BB with respect to Camera frame: " << endl
          //      << T_BB_wrt_camera_frame << endl;

          // Extract position and orientation from velodyne to bounding box tf
          auto T_BB_wrt_lidar_frame = T_camera_wrt_lidar_frame * T_BB_wrt_camera_frame;
          Vector3f position = T_BB_wrt_lidar_frame.block(0, 3, 3, 1);
          position(2) += height / 2; // Correction to z-axis (lidar frame) as centre seems to be on the ground
          // cout << ", " << position(2) << endl;
          Matrix3f rot_mat = T_BB_wrt_lidar_frame.block(0, 0, 3, 3);
          Quaternionf orientation(rot_mat);

          // // DEBUG: Print out info
          // cout << "Computed transformation of BB with respect to Lidar frame: " << endl
          //      << T_BB_wrt_lidar_frame << endl;

          // Check frame in the viewer
          Affine3f T_BB_wrt_lidar_frame_(T_BB_wrt_lidar_frame);
          viewer->addCoordinateSystem(5.0, T_BB_wrt_lidar_frame_, "bb");

          // Create bounding boxes in the viewer
          viewer->addCube(position, orientation, length, width, height, "wire" + to_string(item_count));
          viewer->addCube(position, orientation, length, width, height, "box" + to_string(item_count));

          viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_REPRESENTATION, visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "wire" + to_string(item_count));
          viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_REPRESENTATION, visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "box" + to_string(item_count));
          viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.3, "box" + to_string(item_count)); // slightly transparent box

          if (colour == 1)
          {
            viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "wire" + to_string(item_count));
            viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "box" + to_string(item_count));
          }
          else if (colour == 2)
          {
            viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "wire" + to_string(item_count));
            viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "box" + to_string(item_count));
          }
          else if (colour == 3)
          {
            viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "wire" + to_string(item_count));
            viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "box" + to_string(item_count));
          }
        }
      }
    }

    // Close the file
    TextFile.close();
  }

  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(100ms);
  }

  return 0;
}
