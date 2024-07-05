# LiDAR-Camera-Calibration-Tool

## About
This is a lightweight and easy-to-use LiDAR-camera calibration tool. This work is derived from [lidar_camera_calibration
](https://github.com/heethesh/lidar_camera_calibration), changed into an offline calibration tool and decoupled it from ROS.

Chinese version readme: [激光雷达-相机标定工具](https://zhuanlan.zhihu.com/p/668856220)
## Setup
Install dependencies.
``` bash
#python3.8-3.10
$ pip install opencv-python opend3d
```
Clone the calibration tool to your computer.
``` bash
$ git clone https://github.com/pengpengfei97/LiDAR-Camera-Calibration-Tool.git
```

## Data
- **Solid-state LiDAR:** Get an image file and a pointcloud file with the same timestamp and save them in the data folder.
- **Mechanical LiDAR:** Cause the sparsity of pointcloud generted by mechanical LiDAR, you can use SLAM algorithms to generate dense pointcloud before calibrate. Then save the image and pointcloud files in the data folder. (Note that the coordinate system of the dense pointcloud should be based on the timestamp when the image saved.)

## Usage 

[Youtube demo](https://www.youtube.com/watch?v=kxkrokD8NN4)


1. At the beginning of `calibrate.py`, modify the `IMG_NAME` and `PCD_NAME` to your own file names, and update `CAMERA_MATRIX` and `DIST_COEFFS` with the camera you want to calibrate.
2. Run the following to select more than 5 pairs(>= 6) corresponding 2D points and 3D points for calibration. (Hold down the Shift key while clicking the left mouse button to select 3D points.)
```bash
$ cd LiDAR-Camera-Calibration-Tool/scripts
$ python calibrate.py
```
3. After close the windows, the calibration results will print in the terminal and save in the result folder.
4. Run the `project.py` file to project the pointcloud onto the image and display the calibration results. Refer to step 1 and modify the parameters in `project.py` accordingly.
```bash
$ python project.py
```

## P.S.
Updates will be provided to enhance the tool in the future.