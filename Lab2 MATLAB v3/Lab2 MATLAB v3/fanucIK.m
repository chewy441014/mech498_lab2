%Ian Tomkinson
%Mech 498

%codes the inverse kinematics of the manipulator
function [is_solution,joint_angles]=fanucIK(T,prev_joint_angles,fanuc)

xmin=fanuc.workspace(1);
xmax=fanuc.workspace(2);
ymin=fanuc.workspace(3);
ymax=fanuc.workspace(4);
zmin=fanuc.workspace(5);
zmax=fanuc.workspace(6);

% Shorten variable names
%l_1 = fanuc.parameters.l_1;
l_2 = fanuc.parameters.l_2;
l_3 = fanuc.parameters.l_3;
l_4 = fanuc.parameters.l_4;
l_5 = fanuc.parameters.l_5;
%l_t = fanuc.parameters.l_t
%l_t_rad = fanuc.parameters.l_t_rad

is_solution=true;

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

joint_angles_mat = zeros(4,6);

%theta 1 solution
theta1=atan2(y,x);
joint_angles_mat(:,1) = ones(4,1)*theta1;

%theta 2 solutions
x1=sqrt(x^2+y^2)-l_2;
z1=z;
d=sqrt(l_4^2+l_5^2);

theta2a=pi/2 - (atan2(z1,x1)+acos((x1^2+z1^2+l_3^2-d^2)/(2*l_3*sqrt(x1^2+z1^2))));
theta2b=pi/2 - (atan2(z1,x1)-acos((x1^2+z1^2+l_3^2-d^2)/(2*l_3*sqrt(x1^2+z1^2))));
joint_angles_mat(:,2) = [theta2a, theta2b, theta2a, theta2b]';

%theta 3 solution
theta3a=pi/2 + acos((x1^2+z1^2-l_3^2-d^2)/(2*l_3*d)) - atan2(l_4,l_5);
theta3b=pi/2 - acos((x1^2+z1^2-l_3^2-d^2)/(2*l_3*d)) - atan2(l_4,l_5);
if theta2a > 0
    joint_angles_mat(:,3) = joint_angles_mat(:,3) + [theta3b, 0, theta3b, 0]';
else
    joint_angles_mat(:,3) = joint_angles_mat(:,3) + [theta3a, 0, theta3a, 0]';
end
if theta2b > 0
    joint_angles_mat(:,3) = joint_angles_mat(:,3) + [0, theta3b, 0, theta3b]';
else
    joint_angles_mat(:,3) = joint_angles_mat(:,3) + [0, theta3a, 0, theta3a]';
end

%loop through
for i = 1:2
    joint_angles = joint_angles_mat(i,:);
    
    [~,~,fanuc_T] = fanucFK(joint_angles,fanuc);

    T04=fanuc_T{1}*fanuc_T{2}*fanuc_T{3}*fanuc_T{4};
    R04=T04(1:3,1:3);
    R04i=inv(R04);
    R06=T(1:3,1:3);

    R=R04i*R06;

    %theta 5 solutions
    B1 = atan2(sqrt(R(3,1)^2+R(3,2)^2),R(3,3));
    B2 = atan2(-sqrt(R(3,1)^2+R(3,2)^2),R(3,3));
    if B1 > pi || B1 < 0
        B1 = B2;
    end
    joint_angles_mat(i,5) = B1;
    joint_angles_mat(i+2,5) = B1;

    %theta 4 solutions
    C=atan2(R(2,3)/sin(B1),R(1,3)/sin(B1));
    D=atan2(R(2,3)/sin(B1),R(1,3)/sin(B1))+pi;
    
    %theta 6 solution
    E=atan2(R(3,2)/sin(B1),-R(3,1)/sin(B1));
    F=atan2(R(3,2)/sin(B1),-R(3,1)/sin(B1))+pi;

    %checks for singularity at theta5 =0
    if round(B1,4) == 0
        C = 0;
        E = atan2(-R(1,2),R(1,1));
    elseif round(joint_angles(5),4) == pi || round(joint_angles(5),4) == -pi
        C = 0;
        E = atan2(R(1,2),-R(1,1));
    end
    
    joint_angles_mat(i,4) = C;
    joint_angles_mat(i+2,4) = D;
    joint_angles_mat(i,6) = E;
    joint_angles_mat(i+2,6) = F;
end

joint_limits = zeros(1,6);
for i = 1:6
    joint_limits(i) = fanuc.joint_limits{i}(2);
end

for i = 1:4
    if sum(abs(joint_angles_mat(i,:)) > joint_limits(i)) > 0
        joint_angles_mat(i,:) = ones(1,6)*1000000;
    end
end

%choose solution
norm_sol = zeros(1,4);
for i = 1:4
    norm_sol(i) = norm(joint_angles_mat(i,:)) - norm(prev_joint_angles);
end

best_sol =  min(norm_sol);
if sum(best_sol == norm_sol) > 1
    index = find(best_sol == norm_sol, 1)
else
    index = best_sol == norm_sol
end
joint_angles = joint_angles_mat((index)',:);

%compares solution to limits on robot
for i = 1:length(joint_angles)
    if joint_angles(i) < fanuc.joint_limits{i}(1) || joint_angles(i) > fanuc.joint_limits{i}(2)
        is_solution = false;
        disp(['joint limit ', num2str(i)])
    end
end

joint_angles = round(joint_angles,4)

end