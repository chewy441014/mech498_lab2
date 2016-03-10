%IK_test
x = 0;
prev_joint_angles=[x,x,x,x,x,x];

fanuc=fanucInit;

joint_angles=[0,0,0,0,0,0];

[~,T,~] = fanucFK(joint_angles,fanuc);

[~,joint_angles_new]=fanucIK(T,prev_joint_angles,fanuc);

drawFanuc(joint_angles_new,fanuc);

%%
data = load('box.mat');
s = data.s; % position
c = data.c; % color

pos = s(:,1);
Tnew = eye(4);
Tnew(1,4) = pos(1);
Tnew(2,4) = pos(2);
Tnew(3,4) = pos(3);
Ttool = fanuc.tool{1};
Tnew=inv(Ttool)*Tnew;

[~,joint_angles_new]=fanucIK(Tnew,prev_joint_angles,fanuc);

drawFanuc(joint_angles_new,fanuc);