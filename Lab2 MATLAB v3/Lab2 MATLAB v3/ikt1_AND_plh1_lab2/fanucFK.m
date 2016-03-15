%Ian Tomkinson
%Preston Hill
%Robotics Lab 2
% Mech 498

function [origin,T,fanuc_T] = fanucFK(joint_angles,fanuc)

% Shorten variable names
%l_1 = fanuc.parameters.l_1;
l_2 = fanuc.parameters.l_2;
l_3 = fanuc.parameters.l_3;
l_4 = fanuc.parameters.l_4;
l_5 = fanuc.parameters.l_5;
%l_t = fanuc.parameters.l_t
%l_t_rad = fanuc.parameters.l_t_rad

alpha = [0, pi/2, 0, pi/2, -pi/2, pi/2];
a = [0, l_2, l_3, l_4, 0, 0];
d = [0, 0, 0, l_5, 0, l_4];

T01 = dhtf(alpha(1),a(1),d(1),joint_angles(1));
T12 = dhtf(alpha(2),a(2),d(2),joint_angles(2)+pi/2);
T23 = dhtf(alpha(3),a(3),d(3),joint_angles(3));
T34 = dhtf(alpha(4),a(4),d(4),joint_angles(4));
T45 = dhtf(alpha(5),a(5),d(5),joint_angles(5));
T56 = dhtf(alpha(6),a(6),d(6),joint_angles(6));

T=T01*T12*T23*T34*T45*T56;
fanuc_T={T01, T12, T23, T34, T45, T56};
origin{1} = T01*[0;0;0;1];
origin{2} = T01*T12*[0;0;0;1];
origin{3} = T01*T12*T23*[0;0;0;1];
origin{4} = T01*T12*T23*T34*[0;0;0;1];
origin{5} = T01*T12*T23*T34*T45*[0;0;0;1];
origin{6} = T01*T12*T23*T34*T45*T56*[0;0;0;1];


end
