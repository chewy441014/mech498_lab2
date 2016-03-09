%testfile
fanuc = fanucInit;
x = linspace(0,pi/4,10);
y = zeros(length(x),1);
joint_angles = [x',-x',x',y,y,y];
for i = 1:length(x)
    drawFanuc(joint_angles(i,:),fanuc);
end