%IK_test
% x = 0.5;
% joint_angles=[x,x,x,x,x,x];
% 
% fanuc=fanucInit;
% 
% prev_joint_angles=[0,0,0,0,0,0];
% 
% [~,T,~] = fanucFK(joint_angles,fanuc);
% 
% [~,joint_angles_new]=fanucIK(T,prev_joint_angles,fanuc);
% 
% drawFanuc(joint_angles_new,fanuc);

%%
fanuc=fanucInit;
prev_joint_angles=[0,0,0,0,0,0];
fanuc.brush = 2;
drawFanuc(prev_joint_angles,fanuc);
%%
data = load('box.mat');
s = data.s; % position
c = data.c; % color
fanuc = fanucInit;
prev_joint_angles=[0,0,0,0,0,0];
pos = s(:,1);
T2 = eye(4);
T2(1,4) = pos(1);
T2(2,4) = pos(2);
T2(3,4) = pos(3)+1000;
Ttool = fanuc.tool{1};
T1 = eye(4); T1(3,4) = -1000;
T3 = inv(Ttool)*T2*inv(T1);
disp(pos)
color = [1,0,0];
[~,joint_angles_new]=fanucIK(T3,prev_joint_angles,fanuc)
drawFanuc(joint_angles_new,fanuc);
disp(pos);
%%
data = load('spiral.mat');
fanuc = fanucInit;
s = data.s;
c = data.c;
for t = 1:length(s)
    color = fanuc.brush_colors{c(t)};
    hold on
    plot3(s(1,t),s(2,t),s(3,t),'MarkerEdgeColor',color, 'Marker', '.' ...
        , 'MarkerSize', 18)
end