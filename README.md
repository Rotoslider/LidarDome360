# LidarDome360
## Introduction
The LiDAR Dome 360 App is a powerful tool designed for processing and visualizing LiDAR capture files from Ouster or Velodyne LiDAR Units in the PCAP format. This app specializes in handling data captured when the LiDAR unit is rotated on its side around its axis, providing highly detailed point clouds for analysis.

![Lidar Dome app](https://github.com/Rotoslider/LidarDome360/assets/15005663/8ac00a84-cdde-46ae-8eff-a7f598ea2ca3)

## Features
### Axis of Rotation and Offsets
•	The app allows users to adjust the axis of rotation and offsets (alpha 1, alpha 2, and R) for instances where the LiDAR unit may not be perfectly aligned with the motor's rotation axis.
### Duration and Delay
•	Users can set the duration of the scan per angle and a delay period to ensure stable readings.
•	The LiDAR captures 10 frames per second, and the duration value can be set accordingly (e.g., a value of 600 for a 60-second scan).
### Points to Keep
•	Users have the option to set a "Points to Keep" value, either negative or positive, to filter out unwanted data points.
### Subsampling and Noise Reduction
•	The app includes a "Subsample" option to reduce the data size for faster processing.
•	A "denoise" feature is also available for noise reduction in the point cloud data.
### Bands to Keep
•	This feature allows users to specify the bands or layers of data they wish to retain for analysis.
### Direction
•	The app includes an option to set the motor direction. Incorrect direction may result in distorted output.
### Display Options
•	The final dropdown box, "Display," allows users to visualize the point clouds as they are processed. Options include viewing "All Cycles," only the "First Cycle," or "None" for faster processing.

![Lidar Dome app multi cycles 2](https://github.com/Rotoslider/LidarDome360/assets/15005663/9d0a205a-df48-45ba-b3dc-d50d53003bb0)

## Typical Use Case
### High-Detail Scanning
•	For a high-detail scan, you may perform a 360-degree scan over an extended period, such as several minutes.
### Rapid Scanning
•	For rapid scanning, you might set the RPM to 30, resulting in a scan every 1.5 seconds. For example, a 1-minute capture at 30 RPM with a duration value of 15 and a delay of 10 would yield 38 separate point clouds.
## Installation and Setup
I have created a Gui and a standalone application for it so you do not need MATLAB if you use the installer. You will need to download the MATLAB 2023a Runtime which is freely available at MathWorks: https://www.mathworks.com/products/compiler/matlab-runtime.html The Stand alone was written to run on a Windows 64 machine and must be installed to C:\LidarDome360 or it will not work. You can change this directory in the MATLAB files and recompile if you would like to instal somewhere else.

## Contributing
Contributions to the LiDAR Dome 360 App are welcome! Special thanks to Jason Bula for he started this journey when he produced a paper on creating a low-cost TLS scanner. You can find his GitHub profile here: https://github.com/jason-bula/velodyne_tls
## License
This project is licensed under the MIT License, which allows for open and free use, 

![TLS Touch and Mini Spinny scanners (4)](https://github.com/Rotoslider/LidarDome360/assets/15005663/7e61e6d1-823c-4b54-9a8a-022559970fae)
