%Ian Tomkinson
%Preston Hill
%Robotics Lab 2
%create path file

origin_of_path = [1500; 0; 100];
circ = linspace(0,16*pi,1600);
s=zeros(3,length(circ));
s(1,:) = 100*cos(circ) + origin_of_path(1);
s(2,:) = 100*sin(circ) + origin_of_path(2);
s(3,:) = circ*500/(16*pi) + origin_of_path(3);
L = length(circ);
c = ones(1,L);
c(1:1*L/8) = 1;
c(1*L/8:2*L/8) = 2;
c(2*L/8:3*L/8) = 3;
c(3*L/8:4*L/8) = 4;
c(4*L/8:5*L/8) = 1;
c(5*L/8:6*L/8) = 2;
c(6*L/8:7*L/8) = 3;
c(7*L/8:8*L/8) = 4;

save('lolilolipop.mat')