# manip-env-visu
Barebones repo to visualize simple manipulation environments

### Dependencies

- **Eigen3** - is typically bundled with ubuntu distros, or can be installed via package manager. Otherwise, use the `-DEIGEN_DIR=<eigen_cmake_path>` option when running cmake
- **liburdfdom** - can be installed via package manager, in recent ubuntu distros this does not require ROS
- **VTK** - this can also be installed via package manager, but you will need VTK>=8.2 in order for this code to work. I strongly suggest compiling VTK 8.2 from source and running cmake with the option `-DVTK_DIR=<vtk_cmake_path>`. 
