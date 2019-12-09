# bb_pcl_view (KITTI object format)
Read data from a pcd file and a corresponding txt file.
Display the pointcloud from pcd file.
Create 3d bounding boxes in the same view using information from the text file.
The standard is as generally used for KITTI object dataset.
If the text file is absent, the bounding box for the whole pcd data is presented.

```
$ bash requirements.sh
$ mkdir build
$ cd build
$ cmake ..
$ make
$ ./bb_view <pcdFile> <(optinal) txtFile>
```
