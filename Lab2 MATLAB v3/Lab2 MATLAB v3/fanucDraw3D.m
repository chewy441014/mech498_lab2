function fanucDraw3D( path_file )
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 2 - Inverse Kinematics
%
%    DESCRIPTION - Plot a graphical representation of the FANUC S-500
%    Industrial robot with attached coordinate frames as it moves through a
%    series of poses defined by path_file.
%    
%    ADDITIONAL CODE NEEDED: lots

% Initialize the fanuc struct
fanuc = fanucInit();

% Get path position and color data
data = load(path_file);
s = data.s; % position
c = data.c; % color

% Draw FANUC initially in zero position (do not change)
prev_angles = zeros(1,6);
fanuc.handles = drawFanuc(prev_angles,fanuc);
hold on;

% Draw in 3D
for t = 1:size(s,2)
    
    % Set desired brush color from path file (think about how to handle
    % changes in color)
    fanuc.brush = [];
    ...
    
    % Select desired orientation for the tool (your choice)
    ...
    
    % Set desired position for the tool from path file (not your choice)
    ...
    
    % Solve inverse kinematics for nearest solution
    ...
    
    % Move robot using setFanuc() if solution exists
    ...
    
    % Plot a point at the tool brush tip with the appropriate color
    % (unless the brush selection is zero)
    ...
    
    % Update previous joint angles
    ...

end




end

