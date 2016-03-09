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

joint_angles=[0 0 0 0 0 0];

%Checks whether the transformation is within the workspace
if x>xmax
    is_solution=false;
elseif x<xmin
    is_solution=false;
elseif y>ymax
    is_solution=false;
elseif y<ymin
    is_solution=false;    
elseif z>zmax
    is_solution=false;
elseif z<zmin
    is_solution=false;
end

%theta 1 solution
theta1=atan2(y,x);
joint_angles_cellarr{1} = theta1;

%theta 2 solutions
x1=sqrt(x^2+y^2)-l_2;
z1=z;
d=sqrt(l_4^2+l_5^2);

theta2a=pi/2 - (atan2(z1,x1)+acos((x1^2+z1^2+l_3^2-d^2)/(2*l_3*sqrt(x1^2+z1^2))));
theta2b=pi/2 - (atan2(z1,x1)-acos((x1^2+z1^2+l_3^2-d^2)/(2*l_3*sqrt(x1^2+z1^2))));
joint_angles_cellarr{2} = [theta2a, theta2b];

%theta 3 solution
theta3a=pi/2 - acos((x1^2+z1^2-l_3^2-d^2)/(2*l_3*d)) - atan2(l_4,l_5);
theta3b=pi/2 + acos((x1^2+z1^2-l_3^2-d^2)/(2*l_3*d)) - atan2(l_4,l_5);
joint_angles_cellarr{3} = [theta3a, theta3b];

joint_angles(1) = joint_angles_cellarr{1};
joint_angles(2) = min(abs(joint_angles_cellarr{2} - prev_joint_angles(2)));
joint_angles(3) = min(abs(joint_angles_cellarr{3} - prev_joint_angles(3)));

[~,~,fanuc_T] = fanucFK(joint_angles,fanuc);

T04=fanuc_T{1}*fanuc_T{2}*fanuc_T{3}*fanuc_T{4};
R04=T04(1:3,1:3);
R04i=inv(R04);
R06=T(1:3,1:3);

R=R04i*R06;

%theta 5 solutions
B1=atan2(sqrt(R(3,1)^2+R(3,2)^2),R(3,3));
B2=atan2(-sqrt(R(3,1)^2+R(3,2)^2),R(3,3));
joint_angles_cellarr{5} = [B1,B2];
disp(joint_angles_cellarr{5})

%theta 4 solutions
C=atan2(R(2,3)./sin(joint_angles_cellarr{5}),R(1,3)./sin(joint_angles_cellarr{5}));
D=atan2(R(2,3)./sin(joint_angles_cellarr{5}),R(1,3)./sin(joint_angles_cellarr{5}))+pi;
joint_angles_cellarr{4} = [C, D];
disp(joint_angles_cellarr{4})

%theta 6 solution
E=atan2(R(3,2)./sin(joint_angles_cellarr{5}),-R(3,1)./sin(joint_angles_cellarr{5}));
F=atan2(R(3,2)./sin(joint_angles_cellarr{5}),-R(3,1)./sin(joint_angles_cellarr{5}))+pi;
G=atan2(R(3,2)./sin(joint_angles_cellarr{5}),-R(3,1)./sin(joint_angles_cellarr{5}))-pi;
joint_angles_cellarr{6} = [E, F, G];
disp(joint_angles_cellarr{6})

joint_angles(5) = min(abs(joint_angles_cellarr{5} - prev_joint_angles(5)));
if (joint_angles(5) ~= joint_angles_cellarr{5}(1)) + 1 == 1
    joint_angles(4) = min(abs(joint_angles_cellarr{4}([1,3]) - prev_joint_angles(4)));
    joint_angles(6) = min(abs(joint_angles_cellarr{6}([1,3,5]) - prev_joint_angles(6)));
else
    joint_angles(4) = min(abs(joint_angles_cellarr{4}([2,4]) - prev_joint_angles(4)));
    joint_angles(6) = min(abs(joint_angles_cellarr{6}([2,4,6]) - prev_joint_angles(6)));
end

if round(joint_angles(5),4) == 0
    joint_angles(4) = 0;
    joint_angles(6) = atan2(-R(1,2),R(1,1));
elseif round(joint_angles(5),4) == pi || round(joint_angles(5),4) == -pi
    joint_angles(4) = 0;
    joint_angles(6) = atan2(R(1,2),-R(1,1));
end

%compares solution to limits on robot
for i = 1:length(joint_angles)
    if joint_angles(i) < fanuc.joint_limits{i}(1) || joint_angles(i) > fanuc.joint_limits{i}(2)
        is_solution = false;
    end
end

joint_angles = round(joint_angles,4)

end