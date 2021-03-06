%Ian Tomkinson
%Preston Hill
%Robotics Lab 2


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
prev_joint_angles = zeros(1,6);
fanuc.handles = drawFanuc(prev_joint_angles,fanuc);
hold on;

% Draw in 3D
for t = 1:size(s,2);
    
    
    % Set desired brush color from path file (think about how to handle
    % changes in color)
    fanuc.brush = c(t);
    color = fanuc.brush_colors{c(t)};
    
    % Select desired orientation for the tool (your choice)
    T = eye(4); T(2,2) = -1; T(3,3) = -1; %Orienting z downwards
    
    % Set desired position for the tool from path file (not your choice)
    %shifts back from tool frame to end effector frame
    T6G = fanuc.tool{c(t)};
    T(1:3,4) = [s(1,t),s(2,t),s(3,t)]';
    T06 = T*inv(T6G);

    % Solve inverse kinematics for nearest solution
    [is_solution,joint_angles]=fanucIK(T06,prev_joint_angles,fanuc);
    
    %turns the tool slowly
    if t > 1
        if c(t-1) ~= c(t)
            new_theta1 = linspace(prev_joint_angles(1),joint_angles(1),25);
            new_theta2 = linspace(prev_joint_angles(2),joint_angles(2),25);
            new_theta3 = linspace(prev_joint_angles(3),joint_angles(3),25);
            new_theta4 = linspace(prev_joint_angles(4),joint_angles(4),25);
            new_theta5 = linspace(prev_joint_angles(5),joint_angles(5),25);
            new_theta6 = linspace(prev_joint_angles(6),joint_angles(6),25);
            fanuc.brush = 0;
            for i = 1:size(new_theta1,2)
                if is_solution == true
                    new_angles = [new_theta1(i), new_theta2(i), ...
                        new_theta3(i), new_theta4(i), new_theta5(i),...
                        new_theta6(i)];
                    setFanuc(new_angles,fanuc)
                end
            end
            fanuc.brush = c(t);
        end
    end
    
    % Move robot using setFanuc() if solution exists
    if is_solution == true;
        setFanuc(joint_angles, fanuc);
    else
        disp('The solution does not exist')
    end

    % Plot a point at the tool brush tip with the appropriate color
    % (unless the brush selection is zero)
    plot3(s(1,t),s(2,t),s(3,t)+1000,'MarkerEdgeColor',color, 'Marker', '.' ...
        , 'MarkerSize', 18)
    
    % Update previous joint angles
    prev_joint_angles = joint_angles;

end

end

