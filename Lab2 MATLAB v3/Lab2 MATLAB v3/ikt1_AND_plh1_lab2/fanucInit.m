%Ian Tomkinson
%Preston Hill
%Robotics lab 2


function [ fanuc_struct ] = fanucInit()
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 2 - Inverse Kinematics
%
%    DESCRIPTION - Initialize a structure fanuc_struct to contain important
%    robot information that will be passed into various FANUC simulation
%    functions.
%
%    ADDITIONAL CODE NEEDED:
%
%    Add all dimensions that you will make use of to the nested structure,
%    fanuc_struct.parameters. (Now you never have to remember the actual
%    values!)
%
%    Observe the forward transforms from the end effector to the four
%    different tool brush frames.
%
%    Provide the joint limits from the FANUC data sheet, assuming the
%    ranges are centered at the zero configuration. (Their values are
%    currently all zero.
%
%    Provide the limits of the workspace. (Estimate a box that tightly
%    surrounds the reachable workspace of the robot.)
%

% FANUC dimensions in millimeters
l_1 = 1000; % [mm]
l_2 = 300; % [mm]
l_3=900; %mm
l_4=180; %mm
l_5=1600; %mm
l_6=180; %mm
...




% Tool dimensions in millimeters
l_t_rad = 50; % [mm]
l_t = 300; % [mm]
...





% Fill in FANUC D-H parameters and other necessary parameters 
fanuc_struct.parameters.l_1 = l_1;
fanuc_struct.parameters.l_2 = l_2;
fanuc_struct.parameters.l_3 = l_3;
fanuc_struct.parameters.l_4 = l_4;
fanuc_struct.parameters.l_5 = l_5;
fanuc_struct.parameters.l_6 = l_6;
fanuc_struct.parameters.l_t = l_t;
fanuc_struct.parameters.l_t_rad = l_t_rad;


% FANUC tool brush frames relative to end-effector frame (do not change)
fanuc_struct.tool{1} = makehgtform('zrotate',5*pi/4,...
    'translate',[-l_t_rad,0,0],'yrotate',-pi/4,'translate',[0,0,l_t]);
fanuc_struct.tool{2} = makehgtform('zrotate',7*pi/4,...
    'translate',[-l_t_rad,0,0],'yrotate',-pi/4,'translate',[0,0,l_t]);
fanuc_struct.tool{3} = makehgtform('zrotate',pi/4,...
    'translate',[-l_t_rad,0,0],'yrotate',-pi/4,'translate',[0,0,l_t]);
fanuc_struct.tool{4} = makehgtform('zrotate',3*pi/4,...
    'translate',[-l_t_rad,0,0],'yrotate',-pi/4,'translate',[0,0,l_t]);

% FANUC tool brush selection (0 through 4)
fanuc_struct.brush = 0; % (change this and see what happens)

% FANUC tool brush colors (play with these if you want)
fanuc_struct.brush_colors{1} = 1/255*[255,148,225];
fanuc_struct.brush_colors{2} = 1/255*[173,87,255];
fanuc_struct.brush_colors{3} = 1/255*[111,164,255];
fanuc_struct.brush_colors{4} = 1/255*[126,230,255];

% FANUC base (zero) frame relative to the "station" frame
fanuc_struct.base = makehgtform('translate',[0,0,l_1]);

% FANUC joint limits (deg) (range then speed?)
deg2rad = pi/180;
fanuc_struct.joint_limits{1} = [-150,150]*deg2rad;
fanuc_struct.joint_limits{2} = [-80,80]*deg2rad;
fanuc_struct.joint_limits{3} = [-80,80]*deg2rad;
fanuc_struct.joint_limits{4} = [-240,240]*deg2rad;
fanuc_struct.joint_limits{5} = [-120,120]*deg2rad;
fanuc_struct.joint_limits{6} = [-450,450]*deg2rad;

% Set bounds on the cartesian workspace of the FANUC for plotting in the
% form:  [ xmin, xmax, ymin, ymax, zmin, zmax]
fanuc_struct.workspace = [-2739*cos(30), 2739, -2739, 2739, -1721, 2238];
...

% Set colors to be drawn for each link and associated frame, including the
% tool
fanuc_struct.colors{1} = [1,0,0];
fanuc_struct.colors{2} = [0,1,0];
fanuc_struct.colors{3} = [0,0,1];
fanuc_struct.colors{4} = [1,1,0];
fanuc_struct.colors{5} = [1,0,1];
fanuc_struct.colors{6} = [0,1,1];
fanuc_struct.colors{7} = [0.75,0.75,0.75];



end

