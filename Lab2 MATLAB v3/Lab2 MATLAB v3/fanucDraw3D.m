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
    color = fanuc.brush_colors{c(t)};
    
    % Select desired orientation for the tool (your choice)
    
    
    % Set desired position for the tool from path file (not your choice)
    tool_pos = s(1:3,t);
    T = eye(4); 
    T(1,4) = tool_pos(1); 
    T(2,4) = tool_pos(2); 
    T(3,4) = tool_pos(3);
    T2 = eye(4); T2(3,4) = -1000;
    
    %shifts back from tool frame to end effector frame
    Ttool = fanuc.tool{c(t)};
    T1=inv(Ttool)*T*inv(T2);

    % Solve inverse kinematics for nearest solution

    [is_solution,joint_angles]=fanucIK(T1,prev_joint_angles,fanuc);
    % Move robot using setFanuc() if solution exists

    if is_solution == true;
        setFanuc(joint_angles, fanuc);
        %disp(num2str(t))
    else
        disp('The solution does not exist')
    end

    % Plot a point at the tool brush tip with the appropriate color
    % (unless the brush selection is zero)
    ...
    plot3(s(1,t),s(2,t),s(3,t),'MarkerEdgeColor',color, 'Marker', '.' ...
        , 'MarkerSize', 18)
    %pause(0.25)
    
    % Update previous joint angles
    ...
    prev_joint_angles = joint_angles;

end




end

