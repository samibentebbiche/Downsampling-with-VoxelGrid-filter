## Overview
This repository demonstrates how to downsample a cloud of points using the **VoxelGrid filter** provided by the **Point Cloud Library (PCL)** in C++. The VoxelGrid filter is an efficient method for reducing the density of point clouds by approximating the points in a voxel grid structure, which simplifies processing and reduces memory usage.

![cloud Points](https://github.com/user-attachments/assets/af7fcfdd-9efe-43ba-af16-00928633a711)


---


## Prerequisites
To build and run this project, ensure you have the following:
- **C++ Compiler**: Supporting C++11 or higher.
- **Point Cloud Library (PCL)**: Installed and configured on your system.
- **CMake**: Build system generator for managing the project.

---

## Build
```bash
  mkdir build
  cd build
  cmake ..
  make
```
## Usage
```bash
./voxel_grid file_input.ply [ratio of downsampling]
```
