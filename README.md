# Voaige Depth Registration 

## Project Description

Github to manage the development of Voiage's 3D scene reconstruction source code.

## Useful Repos

Helpful links to get openCV images from both cameras. Follow the instructions below to get the neccessary packages installed for pyrealsense2. Run `realsense-viewer` and add devices to the UI. Open the hamburger menu for each camera to get the SN. 

1. https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md
2. https://github.com/IntelRealSense/librealsense/issues/1735

## Conda Environment Setup

*Note* - SIFT was discontinued for > openCV 3.4 so need to install opencv-contrib-python instead.

1. `pip3 install --upgrade pip` 
2. `conda create -n voaige python=3.8`
3. `conda activate voiage`
4. `pip3 install opencv-python-headless`
5. `pip3 install opencv-contrib-python`
6. `pip3 install sklearn`
7. `pip3 install packaging` 
8. `pip3 install Pillow`
9. `pip3 install six`







