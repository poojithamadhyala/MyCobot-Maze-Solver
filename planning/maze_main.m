clc; clear; close all;
robotURDF = 'mycobot_pro600.urdf';
robot = importrobot(robotURDF);

% Define the specified home configuration
homeConfig = homeConfiguration(robot);
homeConfig(1).JointPosition = deg2rad(46.484);    
homeConfig(2).JointPosition = deg2rad(-120.495);  
homeConfig(3).JointPosition = deg2rad(-107.298);     
homeConfig(4).JointPosition = deg2rad(-44.824);       
homeConfig(5).JointPosition = deg2rad(88.857);    
homeConfig(6).JointPosition = deg2rad(-8.877);

% Read Marker Positions from File
markerPositions = readmatrix('maze_coords1.txt', 'Delimiter', ','); % Read coordinates
numMarkers = size(markerPositions, 1);

% Setup IK Solver
ik = inverseKinematics('RigidBodyTree', robot);
ikWeights = [0.5 0.5 0.5 0.5 0.5 0.5]; % IK weights
ikInitGuess = homeConfig; % Use home configuration as the initial guess

% Define fixed orientation
orientation = eul2quat([0 * pi / 180, 0 * pi / 180, -90 * pi / 180], "XYZ");

% Visualization Setup
figure('units', 'normalized', 'outerposition', [0 0 1 1]); % Full-screen figure
show(robot, homeConfig, 'Frames', 'off', 'PreservePlot', false);
hold on;
plot3(markerPositions(:, 1), markerPositions(:, 2), markerPositions(:, 3), 'ro', 'LineWidth', 2); % Red waypoints
view([230, 10]);
axis auto;
% Adjust the view to focus on the end-effector position
view([0, 90]); % Top-down view
axis equal; % Ensure equal scaling for all axes
xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
grid on; % Enable grid for better visualization

% Process Each Segment Between Markers
numPoints = 15; % Interpolation points per segment
jointAngles = []; % Store joint angles for all trajectory points
for k = 1:(numMarkers - 1)
    % Start and end positions for the current segment
    startPosition = markerPositions(k, :);
    endPosition = markerPositions(k + 1, :);

    % Interpolate between start and end points
    xpositions = linspace(startPosition(1), endPosition(1), numPoints);
    ypositions = linspace(startPosition(2), endPosition(2), numPoints);
    zpositions = linspace(startPosition(3), endPosition(3), numPoints);
    points3D = [xpositions', ypositions', zpositions'];

    % Plot the trajectory (green line)
    plot3(xpositions, ypositions, zpositions, 'g.-');

    % Compute IK for each interpolated point and animate robot movement
    for i = 1:numPoints
        targetPosition = points3D(i, :);

        % Formulate the pose for IK with fixed orientation
        pose = trvec2tform(targetPosition) * quat2tform(orientation);

        % Solve IK
        [configSoln, ~] = ik('Link_6', pose, ikWeights, ikInitGuess);

        % Update IK initial guess
        ikInitGuess = configSoln; % Set the solution as the new guess

        % Extract joint angles (convert from radians to degrees)
        currentJointAngles = cell2mat(arrayfun(@(j) j.JointPosition, configSoln, 'UniformOutput', false));
        jointAngles = [jointAngles; currentJointAngles * 180 / pi];
        writematrix(jointAngles(i, :), 'angles_maze.csv', 'WriteMode', 'append');
        % Animate robot
        show(robot, configSoln, 'Frames', 'off', 'PreservePlot', false);
        title(sprintf('Segment: %d, Point: %d/%d', k, i, numPoints));
        drawnow;
    end
end

% Display Final Joint Angles
disp('Final joint angles (in degrees):');
disp(jointAngles);

disp('Trajectory animation complete.');