# LidarDome360
## Introduction
The LiDAR Dome 360 App is a powerful tool designed for processing and visualizing LiDAR capture files from Ouster or Velodyne LiDAR Units in the PCAP format. This app specializes in handling data captured when the LiDAR unit is rotated on its side around its axis, providing highly detailed point clouds for analysis.

![Lidardome 360 app](https://github.com/Rotoslider/LidarDome360/assets/15005663/bfddc4e0-ac25-4cb1-b349-2b0e2e9c69d6)


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

### Overview of the Lidar Data Processing Program

#### 1. Initialization
- The program starts by loading the input pcap file using the `velodyneFileReader` or `ousterFileReader`object from the Lidar Toolbox, enabling access to individual frames in the pcap file.
- It calculates the total number of frames in the pcap file and determines the number of usable frames.

#### 2. Frame Processing Setup
- Skips the first frames as specified by the 'Delay' box to allow for start and acceleration of the motor.
- Processes the numner of frames per 360-degree cycle based on the 'Duration' box. At 60hz the lidar produces 10 frames per second.
- Calculates the number of full 360-degree cycles possible (`cycles = floor(usableFrames / times)`).

#### 3. Processing Each 360-Degree Cycle
- Loops through each cycle, starting with cycle number 1.
- Within each cycle, it loops through each frame, performing the following actions:
  - Calculates the current frame number based on the cycle number and index.
  - Skips the first first frames as specified.
  - Reads the point cloud for the current frame using `***FileReader`.
  - Applies filtering to keep only points with X > 0 based on the 'pos' parameter.
  - Applies a series of transformation matrices (VM, T4, A2, A1, T) to align and rotate each point cloud frame.
  - Saves the transformed point cloud in a Cell array.

#### 4. Post-Processing
- After processing all frames for a cycle, merges the point clouds into a single point cloud.
- Applies downsampling if a `gridStep` is specified.
- Applies noise filtering based on the 'noise' parameter.
- Rotates the point cloud to orient Z upwards.
- Saves the merged point cloud to a file (e.g., cycle_1_VLP32_3800_frames_merged.ply) in the results folder.

#### 5. Summary
- The pcap file is processed frame-by-frame, transforming each frame and merging them into a single point cloud.
- Noise filtering is applied, and the final merged point cloud is saved to disk.

### Detailed Breakdown of (Frame-by-Frame Processing)
- Calculates the current frame number (`NF`) based on the cycle number, index, and 'first' parameter.
- Reads `ptCloudIn` from the pcap file for frame NF.
- Processes `ptCloudIn` to extract intensity values and apply a binary mask based on X values.
- Sets `curr_angle` based on the angle array and index.
- Initializes `curr_img` to `ptCloudIn` and applies transformations.
- Saves the final transformed point cloud in `Cloud{idx}`.
- Repeats the process for each frame in the cycle to build the Cell array of transformed clouds.

### Explanation of Transformations
- VM matrix rotates the point cloud around the axis that the LiDAR is spinning around (Y-axis).
- T4 matrix adjusts for variation in the rotation speed of the LiDAR motor. This is not broke out in the app but can be changed in the code.
- A2 and A1 matrices apply fixed rotations around the X and Z axes.
- The T matrix translates the entire point cloud up in the vertical direction. This is R in the App and is the offset in meters.
- These transformations realign the raw point cloud into a consistent coordinate system with Z pointing up.

### Band Processing
- Velodyne VLP-32 lidar has 32 laser channels (bands), each providing a slice of the point cloud.
- Processes each band individually to apply transformations and realignment.
- Loops through each frame, extracting and concatenating x, y, z coordinates for each band.
- Applies transformations to the full frame point cloud.
- Saves the transformed point cloud for each frame into a Cell array.
- After processing all frames, it has an array of transformed 32-band point clouds for the entire 360-degree scan.

### Detailed Breakdown of (Band Processing)
- Creates transformation matrix VM based on `curr_angle`.

   The processing happens on a per-band, per-frame basis first, before aggregating across frames:

   1. Take band 1, frame 1. Apply VM transform to the X,Y,Z coordinates from this band/frame.

   2. Take band 1, frame 2. Apply VM transform to X,Y,Z coordinates. 

   3. Repeat for all frames, applying VM transform to band 1 per frame. 

   4. Now aggregate - take the VM-transformed band 1 points from all frames, combine them together into a single point cloud. 

   5. Apply additional corrections like alpha calibration to the aggregated band 1 point cloud.

   6. Repeat steps 1-5 for each band.

So in summary:

   - VM applied on raw data, per band, per frame
   - Aggregate all frames for a given band 
   - Apply additional corrections to aggregated cloud per band
   - End result is N corrected point clouds, one per band



## Contributing
Contributions to the LiDAR Dome 360 App are welcome! Special thanks to Jason Bula for he started this journey when he produced a paper on creating a low-cost TLS scanner. You can find his GitHub profile here: https://github.com/jason-bula/velodyne_tls
## License
This project is licensed under the MIT License, which allows for open and free use, 

![TLS Touch and Mini Spinny scanners (4)](https://github.com/Rotoslider/LidarDome360/assets/15005663/7e61e6d1-823c-4b54-9a8a-022559970fae)
