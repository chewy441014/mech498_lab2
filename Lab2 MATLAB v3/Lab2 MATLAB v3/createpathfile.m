%create path file


circ = linspace(0,2*pi,50);

s=1000*ones(3,length(circ)*2);

s(:,1:length(circ))=s(:,1:length(circ))+[100*cos(circ);100*sin(circ);...
    zeros(1,length(circ))];
s(:,length(circ)+1:2*length(circ))=s(:,length(circ)+1:2*length(circ))+...
    [100*cos(circ);zeros(1,length(circ));100*sin(circ)];


c(:,1:length(circ))=1;
c(:,length(circ)+1:2*length(circ))=2;

save('data')