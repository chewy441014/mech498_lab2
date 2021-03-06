%Ian Tomkinson
%Preston Hill
%Mech 498 Lab 2

%codes the inverse kinematics of the manipulator
function [is_solution,joint_angles]=fanucIK(T,prev_joint_angles,fanuc)

xmin=fanuc.workspace(1);
xmax=fanuc.workspace(2);
ymin=fanuc.workspace(3);
ymax=fanuc.workspace(4);
zmin=fanuc.workspace(5);
zmax=fanuc.workspace(6);

% Shorten variable names
l_2 = fanuc.parameters.l_2;
l_3 = fanuc.parameters.l_3;
l_4 = fanuc.parameters.l_4;
l_5 = fanuc.parameters.l_5;

%initialize variables
is_solution=true;
joint_angles_mat = zeros(4,6);

%end position adjusted for link 5 (rotated according to T)
pos = T(1:3,4) - T(1:3,1:3)*[0; 0; l_4];
x=pos(1);
y=pos(2);
z=pos(3);

%Checks whether the transformation is within the workspace
if x>xmax
    is_solution=false;
    disp('xmax ')
elseif x<xmin
    is_solution=false;
    disp('xmin ')
elseif y>ymax
    is_solution=false;
    disp('ymax ')
elseif y<ymin
    is_solution=false;  
    disp('ymin ')
elseif z>zmax
    is_solution=false;
    disp('zmax ')
elseif z<zmin
    is_solution=false;
    disp('zmin ')
end

%theta 1 solution
%assumption: only one solution
theta1=atan2(y,x);
if round(sin(theta1)*1000)/1000 == 0
    if round(theta1*1000)/1000 ~= 0
        if theta1 > 0
            theta1 = theta1 - pi;
        end
        if theta1 < 0
            theta1 = theta1 + pi;
        end
    end
end
joint_angles_mat(:,1) = ones(4,1)*theta1;

%theta 2 solutions
%geometric constants
x1=sqrt(x^2+y^2)-l_2;
z1=z;
d=sqrt(l_4^2+l_5^2);
beta = atan2(z1,x1);

%assumption: two solutions
trident = acos((x1^2+z1^2+l_3^2-d^2)/(2*l_3*sqrt(x1^2+z1^2)));
if trident < 0
    trident = trident + pi;
end
if trident > pi
    trident = trident - pi;
end
theta2a=(beta + trident) - pi/2;
theta2b=(beta - trident) - pi/2;

%theta 3 solution
trident2 = acos((x1^2+z1^2-l_3^2-d^2)/(2*l_3*d));
theta3a = pi/2 - atan2(l_4,l_5) + trident2;
theta3b = pi/2 - atan2(l_4,l_5) - trident2;
if trident2 > 0
    joint_angles_mat(:,2) = [theta2b, theta2a, theta2b, theta2a]';
else
    joint_angles_mat(:,2) = [theta2a, theta2b, theta2a, theta2b]';
end
joint_angles_mat(:,3) = [theta3a, theta3b, theta3a, theta3b]';

%loop through joint angles matrix (storage for four solutions)
%we are assuming there are only two solutions with theta1,2 and 3
%(elbow up and elbow down)
%I think everything above is correct but I am double double checking theta3
for i = 1:2
    joint_angles = joint_angles_mat(i,:);
    
    T01 = dhtf(0,0,0,joint_angles(1));
    T12 = dhtf(pi/2,l_2,0,joint_angles(2)+pi/2);
    T23 = dhtf(0,l_3,0,joint_angles(3));
    T34 = dhtf(pi/2,l_4,l_5,0);
    R01 = T01(1:3,1:3);
    R12 = T12(1:3,1:3);
    R23 = T23(1:3,1:3);
    R34 = T34(1:3,1:3);
    R04 = R01*R12*R23*R34; 
    R = R04'*T(1:3,1:3);
    
    %theta 5 solutions
    if round(R(3,3)*10000)/10000 ~= 1 || round(R(3,3)*10000)/10000 ~= -1
        B1 = atan2(sqrt(R(3,1)^2+R(3,2)^2),R(3,3));
        B2 = atan2(-sqrt(R(3,1)^2+R(3,2)^2),R(3,3));
        joint_angles_mat(i,5) = B1;
        joint_angles_mat(i+2,5) = B2;

        %theta 4 solutions
        C=atan2(R(2,3)/sin(B1),R(1,3)/sin(B1));
        D=atan2(R(2,3)/sin(B2),R(1,3)/sin(B2));

        %theta 6 solution
        E=atan2(R(3,2)/sin(B1),-R(3,1)/sin(B1));
        F=atan2(R(3,2)/sin(B2),-R(3,1)/sin(B2));

        joint_angles_mat(i,4) = C;
        joint_angles_mat(i+2,4) = D;
        joint_angles_mat(i,6) = E;
        joint_angles_mat(i+2,6) = F;
    else
        %checks for singularity at theta5 =0
        C = 0;
        E = atan2(-R(1,2),R(1,1));
        disp('Singular time')
        joint_angles_mat(i,4) = C;
        joint_angles_mat(i+2,4) = C;
        joint_angles_mat(i,6) = E;
        joint_angles_mat(i+2,6) = E;
    end
end
joint_limits = zeros(1,6);
for i = 1:6
    joint_limits(i) = fanuc.joint_limits{i}(2);
end

%choose solution
norm_sol = zeros(1,4);
for i = 1:4
    norm_sol(i) = norm(joint_angles_mat(i,:)) - norm(prev_joint_angles);
end

best_sol =  min(norm_sol);
if sum(best_sol == norm_sol) > 1
    index = find(best_sol == norm_sol, 1);
else
    index = best_sol == norm_sol;
end
joint_angles = joint_angles_mat((index)',:);

%compares solution to limits on robot
for i = 1:length(joint_angles)
    if joint_angles(i) < fanuc.joint_limits{i}(1) || joint_angles(i) > fanuc.joint_limits{i}(2)
        is_solution = false;
        disp(['joint limit ', num2str(i)])
        disp(joint_angles(i))
    end
end

end