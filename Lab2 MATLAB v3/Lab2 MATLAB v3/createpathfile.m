%create path file

circ = linspace(0,16*pi,10000);
s=zeros(3,length(circ));
s(1,:) = 100*cos(circ) + 750;
s(2,:) = 100*sin(circ);
s(3,:) = circ*500/(16*pi) + 500;
c = ones(1,length(circ)*2);
c(1:length(circ)/4) = 1;
c(length(circ):2*length(circ)/4) = 2;
c(2*length(circ):3*length(circ)/4) = 3;
c(3*length(circ):length(circ)) = 4;

save('spiral.mat')