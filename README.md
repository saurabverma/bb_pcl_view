# bb_pcl_view
Read data from a pcd file and a corresponding txt file.
Display the pointcloud from pcd file.
Create 3d bounding boxes in the same view using information from the text file.

```
$ bash requirements.sh
$ mkdir build
$ cd build
$ cmake ..
$ make
$ ./bb_view <pcdFile> <(optinal) txtFile>
```
