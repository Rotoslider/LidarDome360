
function processLidarData(app, cyc, pos, pos2, puck, spinDirection, times, angle, noise, export, first, alpha_1, alpha_2, gridStep, R, input_file_name, calibFileName)

%% Directory Management
%load('C:\Lidar Dome 360\application\settings.mat')

%browse to file
disp(['Input filename: ', input_file_name]);
disp(['Ouster Calibration filename: ', calibFileName]);

file = input_file_name;
[filepath,~,~] = fileparts(file);
input = filepath;

% Export folder
resultsDir = fullfile(input, 'results');
disp(resultsDir)
try
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end
catch ME
    disp(['Error creating directory: ', ME.message])
end

output =resultsDir;
file = input_file_name;
[~,output_file_name,ext] = fileparts(file);
disp(['Output filename: ', output_file_name]);

%% Initialization of parameters 

% times = Scan duration in seconds
% angle = Rotation in degrees around motor axis. Ouster TLS X+ is up, Velodyne is Y+ up
% first = Time at the first frame (delay)
% puck = version of lidar 1 = 'VLP16', 2 = 'VLP32C', 3 = Ouster
% spinDirection = Clockwise or CounterClockwise

%% Export parameters
% pos set to 1 normally / Set to 0 for calibration of a2
% pos2 set to 1 normally / Set to 0 to keep save First and last band/ Set to 1 to keep all bands/ Set to 2 to keep middle band
% gridStep Set the grid resolution in [m] /  0 will subsample merged files to 0.05

%% Calibration parameters
% Correction angle alpha_1 and alpha_2 [degrees]
alpha_1 = alpha_1 * spinDirection;
alpha_2 = alpha_2 * spinDirection; %Incorrect value causes domeing
% R = Arm length [m] distance rotation center of Lidar off from Motor axis
theta3 = 0; %adjust for irregularities and discrepancies in the speed of the motor

%% Point cloud correction to be applied during rotation
% 
%  In this part of the code, the scan will be extracted frame by frame 
%  in order to to apply the necessary correction to realign the point cloud.
%  First of all, only images containing positive X will be kept, then each
%  image will be processed separately.
%  Two transformation matrices will be applied to each image. One
%  containing rotation according to the LiDAR angle (varies over time) and
%  the other, an applied translation corresponding to the distance between 
%  the arm and the optical center of the LiDAR. This distance was measured
%  manually and corresponds to a displacement over z of R = 0.0945m (can
%  be improved). Finally, each image is saved separately in a Cell.

%% Initialisation
previous_cycle = 0;
if cyc ~= 0
hFig = figure; % Create a figure handle
end
filesToMerge = {}; % Reset the list of files to merge
ptCloudIn = {};

% Determine which reader to use based on the puck value
switch puck
    case 1
        % VLP16
        veloReader = velodyneFileReader(input_file_name, 'VLP16');
        totalScanFrames = veloReader.NumberOfFrames; % count frames of scan
        readerUsed = 'Velodyne';
    case 2
        % VLP32C
        veloReader = velodyneFileReader(input_file_name, "VLP32C");
        totalScanFrames = veloReader.NumberOfFrames; % count frames of scan
        readerUsed = 'Velodyne';

    case 3
        % OUSTER
        ousterReader = ousterFileReader(input_file_name,calibFileName);
        totalScanFrames = ousterReader.NumberOfFrames; % count frames of scan for Ouster
        readerUsed = 'OUSTER';
    otherwise
        error('Invalid puck value. Please use 1 for VLP16, 2 for VLP32C, or 3 for OUSTER.');
end
fprintf('Reader Used: %s\n', readerUsed);
fprintf('Total Number of Scan Frames (totalScanFrames): %d\n', totalScanFrames);


usableFrames = totalScanFrames - first;  % frames-start delay
fprintf('Total Number of Scan Frames minus Delay (usableFrames): %d\n', usableFrames);

% Check if 'times' exceeds the usable times
if times > usableFrames
    warning('Entered time value is too large. It has been replaced with the maximum allowed Time of %d.', usableFrames);
    times = usableFrames;  % replace times with maximum allowed value
  
end

fprintf('Frames per angle: %d\n', times);

settings_file = ('C:\LidarDome360\application\settings.mat');
save(settings_file, 'cyc', 'angle', 'times', 'first', 'gridStep', 'alpha_1', 'alpha_2', 'R', 'pos', 'pos2', 'spinDirection', 'input_file_name', 'calibFileName', 'puck', 'usableFrames', 'totalScanFrames')

total_angle = angle; % Initial total angle
%last = first + times; % Time at the last frame
angle_deg=(0:total_angle/times:angle) * spinDirection; % Angle after each frame
angle_array = deg2rad(angle_deg); % Angle in radian
s = 0; 

numberOfFrames = times; 
Cloud = cell(1, numberOfFrames);
cycles = floor(usableFrames / times);
current_start_frame = first;



%% START of the outer loop (cycles loop):
for current_cycle = 1:cycles
    filesToMerge = {}; % reset the array at each loop
    pointCloudsForCurrentCycle = {};
    ptCloudIn = {};

    %disp(['Processing cycle ', num2str(current_cycle), ' starting from frame ', num2str(current_start_frame)]); % Debug info

    % Insert the stop check here
    if app.stopProcessing
        % Update the progress field to notify the user
        app.ProgressField.Value = 'Processing stopped. Press Run to start again.';
        break;  % Exit the loop
    end

    filename_cycle_prefix = sprintf('cycle_%d_', current_cycle);

    s = 0; % Reset s for each cycle

    % Process individual frames for the current 360-degree cycle
    for idx = 1:times

        if app.stopProcessing
            % Update the progress field to notify the user
            app.ProgressField.Value = 'Processing stopped. Press Run to start again.';
            break;  % Exit the loop
        end

        current_frame = current_start_frame + idx - 1; % Calculate the current frame

        % Ensure we're not exceeding the total number of frames
        if current_frame > totalScanFrames
            disp('Breaking out of the loop early! exceeding the total number of frames');
            break;
        end

        app.ProgressField.Value = ['Processing frame ', num2str(idx), ' of ', num2str(times), ', cycle ', num2str(current_cycle), ' of ', num2str(cycles)];
        drawnow;

%% Initialize Cloud
% Use the value of 'idx' to access the 'angle' array
curr_angle = angle_array(idx);

NF = current_start_frame + idx - 1;

disp(['Reading frame ', num2str(NF)]); % Debug info for frame reading

if puck == 1 || puck == 2  % For VLP16 and VLP32C
    ptCloudIn = readFrame(veloReader, NF);
elseif puck == 3  % For OUSTER
    ptCloudIn = readFrame(ousterReader, NF);
else
    error('Invalid puck value. Please use 1 for VLP16, 2 for VLP32C, or 3 for OUSTER.');
end


%%  Filter out front or back half of laser data
% pos 1 filters (velodyne) the point cloud data to only include points that are
% located in the positive X half-space (X > 0). Any point cloud data with
% X-coordinate values less than or equal to zero are removed.
if app.stopProcessing
    app.ProgressField.Value = 'Processing stopped. Press Run to continue.';
    return;  % Exit the function
end

if pos == 1

    if puck == 3  % Ouster logic
        ptCloudIn3 = ptCloudIn.Location(:,:,2); % extract the y-coordinate values from the point cloud (ouster)
    else  % Velodyne logic for puck 1 or 2
        ptCloudIn3 = ptCloudIn.Location(:,:,1); % extract the x-coordinate values from the point cloud (velodyne)
    end

    ptCloudIn3(isnan(ptCloudIn3))=0; % replacing all the NaN values in the x-coordinates with zeros

    ptCloudIntensity = ptCloudIn.Intensity(:,:); % intensity values of the point cloud are being extracted
    ptCloudIntensity(isnan(ptCloudIntensity))=0; % any NaN values are being replaced with zeros

    ptCloudIn3(ptCloudIn3<0)=0; % intensity values are being multiplied by the binary mask
    ptCloudIn3(ptCloudIn3>0)=1; % zeroes out the intensity values for all points with an x-coordinate less than or equal to zero

    ptCloudIntensity2 = single(ptCloudIntensity(:,:)).*ptCloudIn3; % intensity values are being multiplied by the binary mask
    ptCloudIn = pointCloud(ptCloudIn.Location(:,:,:).*ptCloudIn3,'Intensity',ptCloudIntensity2);
end
    % the pos variable determines which half-space of the relevant axis
    % (Velodyne) X-axis or (Ouster) Y-axis is preserved in the output point cloud. This
	% filtering process is accomplished by creating a binary mask based on the
	% sign of the coordinate values and applying this mask to the point cloud
	% coordinates and intensity values. The output point cloud only includes
	% points from the specified half-space and the rest of the points are
	% filtered out.


% puck 1 or 2 and pos 2 (velodyne) filters the point cloud data to only include points that are
% located in the negative X half-space (X < 0). Any point cloud data with
% X-coordinate values greater than or equal to zero are removed.

% puck 3 and pos 2 (ouster) filters the point cloud data to only include points that
% are located in the negative Y half-space (Y < 0). Any point cloud data
% with Y-coordinate values greater than or equal to zero are removed.
if app.stopProcessing
    app.ProgressField.Value = 'Processing stopped. Press Run to continue.';
    return;  % Exit the function
end

if pos == 2
    if puck == 3  % Ouster logic
        ptCloudIn3 =  ptCloudIn.Location(:,:,2); % extract the y-coordinate values from the point cloud (ouster)
    else  % Velodyne logic for puck 1 or 2
        ptCloudIn3 = ptCloudIn.Location(:,:,1); % extract the x-coordinate values from the point cloud (velodyne)
    end
    ptCloudIn3(isnan(ptCloudIn3))=0; % replacing all the NaN values in the y-coordinates with zeros

    ptCloudIntensity = ptCloudIn.Intensity(:,:);
    ptCloudIntensity(isnan(ptCloudIntensity))=0;

    ptCloudIn3(ptCloudIn3>0)=0;
    ptCloudIn3(ptCloudIn3<0)=1;

    ptCloudIntensity2 = single(ptCloudIntensity(:,:)).*ptCloudIn3;
    ptCloudIn = pointCloud(ptCloudIn.Location(:,:,:).*ptCloudIn3,'Intensity',ptCloudIntensity2);
end

clear ptCloudIn2 ptCloudIn3 


%% Transformation of each band
% Transformation matrix

if puck == 3  % Alignment correction as a function of motor speed (x-axis) (ouster)
    VM = [1 0 0 0; 0 cos(curr_angle) sin(curr_angle) 0 ; ...
        0 -sin(curr_angle) cos(curr_angle) 0 ; 0 0 0 1];
else  % Alignment correction as a function of motor speed (y-axis) (velodyne)
    VM = [cos(curr_angle) 0 sin(curr_angle) 0; 0 1 0 0; -sin( curr_angle) 0 ...
        cos(curr_angle) 0; 0 0 0 1];
end

% Alpha_1 correction (Y-axis)
A1 = [1 0 0 0; 0 cosd(alpha_1) sind(alpha_1) 0 ; ...
    0 -sind(alpha_1) cosd(alpha_1) 0 ; 0 0 0 1];

% Alpha_2 correction (z-axis)
A2 =  [cosd(alpha_2) sind(alpha_2) 0 0; ...
    -sind(alpha_2) cosd(alpha_2) 0 0; 0 0 1 0; 0 0 0 1];

% R offset correction to translate away from rotation axis (velodyne and ouster)
if puck == 3 
    T = [1 0 0 0; 0 1 0 R; 0 0 1 0; 0 0 0 1];  % Ouster optical center correction in meters offset by 0.02303 in the positive Y-direction relative to the X-axis
else
    T = [1 0 0 R; 0 1 0 0; 0 0 1 0; 0 0 0 1];  % Velodyne optical center correction in meters offset by 0.0424 in the positive X direction relative to the Y-axis
end    

% Speed adjustement
if puck == 3 % Speed adjustment (x-axis) (ouster)
    %T4 = [cosd(theta3) 0 -sind(theta3) 0; 0 1 0 0; sind(theta3) 0 cosd(theta3) 0; 0 0 0 1];
    T4 = [1 0 0 0;
      0 cosd(theta3) -sind(theta3) 0;
      0 sind(theta3) cosd(theta3) 0;
      0 0 0 1];
else % Speed adjustement (velodyne)
    T4 = [cosd(theta3) sind(theta3) 0 0;
      -sind(theta3) cosd(theta3) 0 0;
      0 0 1 0;
      0 0 0 1];    
end

% Apply transformation

% Start with the original point cloud
curr_img = ptCloudIn;

% Apply VM transformation
tform_R = affinetform3d(VM);
curr_img = pctransform(curr_img, tform_R);

% Apply T4 transformation
tform_T4 = affinetform3d(T4);
curr_img = pctransform(curr_img, tform_T4);

% Apply A2 transformation
tform_A2 = affinetform3d(A2);
curr_img = pctransform(curr_img, tform_A2);

% Apply A1 transformation
tform_A1 = affinetform3d(A1);
curr_img = pctransform(curr_img, tform_A1);

% Apply T transformation
tform = affinetform3d(T);
Cloud{idx} = pctransform(curr_img, tform);  % Final transformed cloud saved in the cell

s = s + 1; % count update
    end

%% Initialisation of export parameters

if pos2 == 1 && puck == 1
    bandNumber = 16; %selects all VLP16 segments
    disp('Selected all VLP16 segments.');
    app.ProgressField.Value = 'Processing all VLP16 segments.';
end

if pos2 == 1 && (puck == 2 || puck ==3 )
    bandNumber = 32; %selects all Ouster or VLP32C segments
    disp('Selected all Ouster or VLP32C segments.');
    app.ProgressField.Value = 'Processing all 32 segments.';

end
if pos2 == 0
    bandNumber = 2; %selects first and last segment
    disp('Selected first and last segment.');
    app.ProgressField.Value = 'Processing First and Last segments.';

end
if pos2 == 2
    bandNumber = 1; %selects middle segment
    disp('Selected middle segment.');
    app.ProgressField.Value = 'Processing all middle segment.';

end

%disp('Finished Initialization of export parameters.');


%% Point cloud merging 

if app.stopProcessing
    app.ProgressField.Value = 'Processing stopped. Press Run to continue.';
    return;  % Exit the function
end

for bande_sep = 1 : bandNumber
    
if pos2 == 1 
    iiii=bande_sep;
end

if pos2 == 0  && puck == 1  % VLP16 selects first and last laser bands
    Bande_calibration = [1 16]; 
    iiii = Bande_calibration(bande_sep);
end

if pos2 == 0  && (puck == 2 || puck ==3 ) % Ouster and VLP32 selects first and last laser bands
    Bande_calibration = [1 32]; 
    iiii = Bande_calibration(bande_sep);
end

if pos2 == 2  && puck == 1  % VLP16 slelects middle laser band
    Bande_calibration = 8; 
    iiii = Bande_calibration(bande_sep);
end

if pos2 == 2  && (puck == 2 || puck ==3 ) % Ouster and VLP32 selects middle laser bands 
    Bande_calibration = 16;
    iiii = Bande_calibration(bande_sep);
end

%% Variable initialization
x = [];
y = [];
z = [];
int = [];

X_ref = [];
Y_ref = [];
Z_ref = [];
int_ref = [];

X_ref_final = [];
Y_ref_final = [];
Z_ref_final = [];
int_ref_final = [];

% Acceleration of the process by combining several loops
if app.stopProcessing
    app.ProgressField.Value = 'Processing stopped. Press Run to continue.';
    return;  % Exit the function
end

for iii = 1 : 10
    for ii = round((times/10*iii)-((times/10)-1)) : round(iii*times/10)


        for i = iiii:iiii % save the band separately

            % Deleting old values
            x1 = [];
            y1 = [];
            z1 = [];
            int1 = [];
            % Selection of points in the correct matrix locations
            % Point clouds are recorded as follows: 16*1800*3
            % 16 corresponds to the band, 1800 corresponds to the number of points recorded
            % per band, 3 corresponds to the x, y and z values.

            x1(i,:) = Cloud{1,ii}.Location(i,:,1);
            y1(i,:) = Cloud{1,ii}.Location(i,:,2);
            z1(i,:) = Cloud{1,ii}.Location(i,:,3);
            int1(i,:) = Cloud{1,ii}.Intensity(i,:);


            x = [x x1(i,:)];
            y = [y y1(i,:)];
            z = [z z1(i,:)];
            int = [int int1(i,:)];


        end
        X_ref = [X_ref x];
        Y_ref = [Y_ref y];
        Z_ref = [Z_ref z];
        int_ref = [int_ref int];

        x = 0;
        y = 0;
        z = 0;
        int = 0;

    end
    X_ref_final = [X_ref_final X_ref];
    Y_ref_final = [Y_ref_final Y_ref];
    Z_ref_final = [Z_ref_final Z_ref];
    int_ref_final = [int_ref_final int_ref];

    %disp(iii) % extraction progress meter
    X_ref = 0;
    Y_ref = 0;
    Z_ref = 0;
    int_ref = 0;

    
end

%% Reconstruction of the point cloud
ref = [X_ref_final; Y_ref_final; Z_ref_final]';
PC_corr1 = pointCloud(ref,'Intensity',int_ref_final');

if gridStep == 0
    PC_downsampled_1 = PC_corr1;
else
    PC_downsampled_1 = pcdownsample(PC_corr1,'gridAverage',gridStep);
end

%% Denoise the point cloud (default values)
%For a dense point cloud with small-scale noise, you might increase NumNeighbors and decrease Threshold.
%For a point cloud with occasional large outliers, you might decrease NumNeighbors and increase Threshold.
%PC_denoised = pcdenoise(PC_downsampled_1);
% if noise == 2
% NumNeighbors = 6; % Similar to CloudCompare's setting
% Threshold = 1; % Start with this value to mimic CloudCompare's standard deviation

if noise == 2
    NumNeighbors = 6; % Similar to CloudCompare's setting
    Threshold = 1; % Start with this value to mimic CloudCompare's standard deviation
    PC_denoised = pcdenoise(PC_downsampled_1, 'NumNeighbors', NumNeighbors, 'Threshold', Threshold);
    [PC_Final1,indices]= removeInvalidPoints(PC_denoised);
    
elseif noise == 1
    NumNeighbors = 3; % Similar to CloudCompare's setting
    Threshold = 1; % Start with this value to mimic CloudCompare's standard deviation
    PC_denoised = pcdenoise(PC_downsampled_1, 'NumNeighbors', NumNeighbors, 'Threshold', Threshold);
    [PC_Final1,indices]= removeInvalidPoints(PC_denoised);
    
else
    [PC_Final1,indices]= removeInvalidPoints(PC_downsampled_1);
end

%% Rotate so Z is up
if puck == 3 % Create the rotation matrix (ouster)
    RotY = [cosd(90) 0 -sind(90) 0; 0 1 0 0; sind(90) 0 cosd(90) 0; 0 0 0 1];
    % Apply the rotation to point cloud to align Z axis upwards (ouster)
    PC_Final1 = pctransform(PC_Final1, affinetform3d(RotY'));
else
    %Create the rotation matrix (velodyne)
    RotX = [1 0 0 0; 0 cosd(90) sind(90) 0; 0 -sind(90) cosd(90) 0; 0 0 0 1];
    % Apply the rotation to point cloud to align Z axis upwards (velodyne)
    PC_Final1 = pctransform(PC_Final1, affinetform3d(RotX'));
end

%% Point cloud export individual files
if pos2 == 1 && export == 1

  disp('Exporting individual files...');

    % Construct file name
    filename = sprintf('%s_%s_%d.ply', filename_cycle_prefix, output_file_name, i);

    % Save point cloud
    pcwrite(PC_Final1,fullfile(output, filename), 'PLYFormat','binary');
end

if export ~= 1
   disp('Individual files not saved'); 
end

%% Point cloud export Merged Files

% Defining output document names WITH adjusted filename_prefix
assert(ischar(output_file_name) || isstring(output_file_name), 'output_file_name is not a string or character array!');
if app.stopProcessing
    app.ProgressField.Value = 'Processing stopped. Press Run to continue.';
    return;  % Exit the function
end
% Initialize the cell array at the beginning of each cycle
if current_cycle ~= previous_cycle
    pointCloudsForCurrentCycle = {};
end

previous_cycle = current_cycle;

% Add the current point cloud to the cell array
%pointCloudsForCurrentCycle{end+1} = PC_Final1;

if app.stopProcessing
    app.ProgressField.Value = 'Processing stopped. Press Run to continue.';
    return;  % Exit the function
end
if ~isempty(PC_Final1)
    pointCloudsForCurrentCycle{end+1} = PC_Final1;
else
    disp('Warning: PC_Final1 is empty!');
end

% Save individual bands if pos2 is set to First/Last or Middle
if pos2 == 0 && (iiii == 1 || iiii == 16 || iiii == 32) || ...
   pos2 == 2 && (iiii == 8 || iiii == 16)
    % Define the filename
    filename = sprintf('%s_%s_%d.ply', filename_cycle_prefix, output_file_name, iiii);
    % Save the current band
    pcwrite(PC_Final1, fullfile(output, filename), 'PLYFormat', 'binary');
end

% Return to input directory
cd(input)

ref = []; % suppression of the loaded point cloud

end

%% Setup for Merge of Files
% Define merge tolerance

if gridStep ~= 0
    mergeTolerance = gridStep;
else
    mergeTolerance = 0.005;
end

if app.stopProcessing
    app.ProgressField.Value = 'Processing stopped. Press Run to continue.';
    return;  % Exit the function
end

% Check if there are multiple point clouds to merge
if length(pointCloudsForCurrentCycle) > 1
    % Initialize the merged point cloud with the first point cloud from the cell array
    mergedPointCloud = pointCloudsForCurrentCycle{1};

    % Merge each subsequent point cloud from the cell array
    for i = 2:length(pointCloudsForCurrentCycle) % Note: start from 2 since we already took the first point cloud
        tempPC = pointCloudsForCurrentCycle{i};
        mergedPointCloud = pcmerge(mergedPointCloud, tempPC, mergeTolerance);
    end

    % Define the output filename WITH adjusted filename_cycle_prefix for the merged file
    merged_filename = sprintf('%s%s_merged.ply', filename_cycle_prefix, output_file_name);

    % Save the merged point cloud
    pcwrite(mergedPointCloud, fullfile(output, merged_filename), 'PLYFormat', 'binary');


    % Set the maximum limit of points for visualization
    maxNumPoints = 5e5; % 100,000 points would be 1e5, 500,000 would be 5e5 and 1,000,000 is to many points.
    if mergedPointCloud.Count > maxNumPoints
        % Downsample the point cloud if it contains more than the maximum limit
        mergedPointCloud = pcdownsample(mergedPointCloud, 'random', maxNumPoints / mergedPointCloud.Count);
    end

    % Normalize the Z values to the range [0, 1]
    z = mergedPointCloud.Location(:, 3); % Z values
    z = (z - min(z)) / (max(z) - min(z)); % normalization to [0, 1]

    % Apply the colormap
    cmap = jet(256); % colormap
    c = round(z * (size(cmap, 1) - 1)) + 1; % Match color indices to z values
    color = uint8(cmap(c, :) * 255); % Convert to 8-bit RGB color

    % Create a new point cloud with color
    mergedPointCloudColor = pointCloud(mergedPointCloud.Location, 'Color', color);
    if (cyc == 2) || (cyc == 1 && current_cycle == 1)
    %if cyc == 2
    figure(hFig); % Bring the dedicated figure to the front
    pcshow(mergedPointCloudColor);
    title(['Merged Point Cloud - Cycle ', num2str(current_cycle)]);
    xlabel("X");
    ylabel("Y");
    zlabel("Z");
    drawnow; % Force immediate update of the figure
    end

    disp(['Merged File is Saved: ', merged_filename]);
    app.PcapFileEditField.Value = ['Merged File Saved: ', merged_filename];

    % Optionally add a pause or wait for a keypress if you want to manually advance through visualizations
    % pause;
else
    disp('Only one file, skipping merging process.')
    if ~isempty(pointCloudsForCurrentCycle)
        singlePointCloud = pointCloudsForCurrentCycle{1};
    else
        disp('Warning: pointCloudsForCurrentCycle is empty!');
        %continue; % or handle it appropriately
    end

    % Normalize the Z values to the range [0, 1]
    z = singlePointCloud.Location(:, 3); % Z values
    z = (z - min(z)) / (max(z) - min(z)); % normalization to [0, 1]

    % Apply the colormap
    cmap = jet(256); % colormap
    c = round(z * (size(cmap, 1) - 1)) + 1; % Match color indices to z values
    color = uint8(cmap(c, :) * 255); % Convert to 8-bit RGB color

    % Create a new point cloud with color
    singlePointCloudColor = pointCloud(singlePointCloud.Location, 'Color', color);

    % Display the single point cloud
    if cyc == 2 || cyc == 1
    figure(hFig); % Bring the dedicated figure to the front
    pcshow(singlePointCloudColor);
    title('Single Point Cloud');
    xlabel("X");
    ylabel("Y");
    zlabel("Z");
    drawnow; % Force immediate update of the figure
    end
end

% Return to input directory
cd(input)

% Check if the frames required for the next cycle breach the usableFrames limit
if (current_start_frame + times - 1) > totalScanFrames
    disp(['Stopping at cycle ', num2str(current_cycle), ' due to reaching the limit of usable frames.']);
    break; %exit the loop
end

% Update the current_start_frame for the next cycle
current_start_frame = current_start_frame + times;
%disp(['Updated start frame for next cycle: ', num2str(current_start_frame)]);
end
%disp(['Final current_start_frame: ', num2str(current_start_frame)]);
%adjustedTimes = times;
end
