# 16-833_Project_MSCKF_VIO

This project is based on the MSCKF implementation by [rohiitb](https://github.com/rohiitb/msckf_vio_python),which in itself is translated from the official C++ implementation [KumarRobotics/msckf_vio](https://github.com/KumarRobotics/msckf_vio),  with the following modifications:
- Changed visualizer
- Improved outlier detection methods (RANSAC)
- Improved feature pruning
- Analysis of changes on pose estimation accuracy

## Vizualizer (viewer.py)
The Pangolin visualizer, originally intended for this project, presents several challenges:

- The Python binding `pypangolin` is not available through pip and requires a complex CMake installation.
- The CMake installation file contains outdated syntax, causing errors during setup.
- `pypangolin` has compatibility issues with newer versions of essential packages like `opencv-python` and `open3d`.
- Downgrading these packages leads to further conflicts, making a stable setup difficult to achieve.

To address these issues, the project now uses a custom `viewer.py` file implemented with `matplotlib` and `opencv-python`. This solution provides similar visualization capabilities without the installation and compatibility problems of `pypangolin`.

3D trajectory of EuRoC MH01 
![Screenshot 2024-12-01 224207](https://github.com/user-attachments/assets/3dbc7303-a64c-480c-bc5e-1c41cba97602)





This work was completed as part of 16-833 Robot Localization and Mapping at Carnegie Mellon University.
