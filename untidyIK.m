clc;
clear;

syms a b c container_size;
% Initialize the dimensions of the container (a cube) and the rectangular box
c_size = container_size; % Side length of the cubic container
l = a; % Length of the rectangular box
w = b;  % Width of the rectangular box
h = c; % Height of the rectangular box
hl = l/2;
hw = w/2;
hh = h/2;

cv1 = [c_size;c_size;c_size];
cv2 = [c_size ; 0 ; c_size];
cv3 = [0;0;c_size];
cv4 = [0;c_size;c_size];
cv5 = [c_size;c_size;0];
cv6 = [c_size;0;0];
cv7 = [0;0;0];
cv8 = [0;c_size;0];

CV = [cv1 cv2 cv3 cv4 cv5 cv6 cv7 cv8];

av1 = [hl ; hw ; hh];
av2 = [hl ; -hw ; hh];
av3 = [-hl ;-hw ; hh];
av4 = [-hl ; hw ; hh];
av5 = [hl ; hw ; -hh];
av6 = [hl ; -hw ; -hh];
av7 = [-hl ;-hw ; -hh];
av8 = [-hl ; hw ; -hh];


MAV= [av1 av2 av3 av4 av5 av6 av7 av8];

syms alpha beta gamma;


WRM = eul2rotm([alpha beta gamma] , "ZYX");

syms x y z ;

WTM = [x;y;z];


pose = trvec2tform(WTM)*WRM;

exMAV = [MAV;1 1 1 1 1 1 1 1];
exWAV = pose*exMAV;
WAV = exWAV(1:3,:);

Q = sqrt(sum((CV - WAV).^2, 1))'