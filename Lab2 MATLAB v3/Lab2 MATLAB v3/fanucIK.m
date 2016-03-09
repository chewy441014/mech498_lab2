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
joint_angles(1)=atan2(y,x);

%theta 2 solutions
x1=sqrt(x^2+y^2)-l_2;
z1=z;
d=sqrt(l_4^2+l_5^2);

theta2a=pi/2 - (atan2(z1,x1)+acos((x1^2+z1^2+l_3^2-d^2)/(2*l_3*sqrt(x1^2+z1^2))));
theta2b=pi/2 - (atan2(z1,x1)-acos((x1^2+z1^2+l_3^2-d^2)/(2*l_3*sqrt(x1^2+z1^2))));

%select the closer solution
if abs(theta2a-prev_joint_angles(2))<abs(theta2b-prev_joint_angles(2))
    joint_angles(2)=theta2a;
else
    joint_angles(2)=theta2b;
end

%theta 3 solution
joint_angles(3)=pi/2 - acos((x1^2+z1^2-l_3^2-d^2)/(2*l_3*d)) - atan2(l_4,l_5);

[~,T,fanuc_T] = fanucFK(joint_angles,fanuc);

T04=fanuc_T{1}*fanuc_T{2}*fanuc_T{3}*fanuc_T{4};
R04=T04(1:3,1:3);
R04i=inv(R04);
R06=T(1:3,1:3);

R=R04i*R06;

%theta 5 solutions
B1=atan2(sqrt(R(3,1)^2+R(3,2)^2),R(3,3));
B2=atan2(-sqrt(R(3,1)^2+R(3,2)^2),R(3,3));

if abs(B1-prev_joint_angles(5))<abs(B2-prev_joint_angles(5))
    joint_angles(5)=B1;
else
    joint_angles(5)=B2;
end

B = joint_angles(5);

%theta 4 solutions
C=atan2(R(2,3)/sin(B),R(1,3)/sin(B));
D=atan2(R(2,3)/sin(B),R(1,3)/sin(B))+pi;

%compares theta 4 solutions
if abs(C-prev_joint_angles(4))<abs(D-prev_joint_angles(4))
    joint_angles(4)=C;
else
    joint_angles(4)=D;
end

%theta 6 solution
E=atan2(R(3,2)/sin(B),-R(3,1)/sin(B));
F=atan2(R(3,2)/sin(B),-R(3,1)/sin(B))+pi;

%compares theta 6 solutions
if abs(E-prev_joint_angles(4))<abs(F-prev_joint_angles(4))
    joint_angles(6)=E;
else
    joint_angles(6)=F;
end

%compares solution to limits on robot
for i = 1:length(joint_angles)
    if joint_angles(i) < fanuc.joint_limits{i}(1) || joint_angles(i) > fanuc.joint_limits{i}(2)
        is_solution = false;
    end
end

joint_angles = round(joint_angles,4);

end