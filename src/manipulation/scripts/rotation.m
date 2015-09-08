%%
clear all
clc
%% rotation matrix to quaternion
alpha = 30/180*pi;
theta = 10/180*pi + pi/2;
Rx = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)]
Rz = [cos(alpha) -sin(alpha) 0;sin(alpha) cos(alpha) 0; 0 0 1]

R=Rz*Rx;

w = sqrt(1+R(1,1)+R(2,2)+R(3,3)) /2
x = (R(3,2)-R(2,3)) / (4*w)
y = (R(1,3)-R(3,1)) / (4*w)
z = (R(2,1)-R(1,2)) / (4*w)

%% quaternion 
angle = 30/180*pi;
axis = [0 0 1];

w = cos(angle/2)
x = axis(1)*sin(angle/2)
y = axis(2)*sin(angle/2)
z = axis(3)*sin(angle/2)


