%Ian Tomkinson
%Preston Hill
%Robotics Lab 2, Q1

%Generates the Transfer function from DH parameters
%angles are assumed to be in radians

function T = dhtf(alpha,a,d,theta)

T= [cos(theta), -sin(theta), 0, a; 
    sin(theta)*cos(alpha), cos(theta)*cos(alpha), ...
    -sin(alpha), -sin(alpha)*d; 
    sin(theta)*sin(alpha), cos(theta)*sin(alpha), ...
    cos(alpha), cos(alpha)*d; 
    0, 0, 0, 1];



end
