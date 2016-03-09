%IK_test

prev_joint_angles=[0,0,0,0,0,0];

fanuc=fanucInit;

joint_angles=[pi/4,pi/4,pi/4,0,0,0];

[~,T,~] = fanucFK(joint_angles,fanuc);

[is_solution,joint_angles_new]=fanucIK(T,prev_joint_angles,fanuc);

drawFanuc(joint_angles_new,fanuc)