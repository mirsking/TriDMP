clear
close all
clc


dt =0.001;
t=0:dt:1;
x = 2*t.^2+cos(4*pi*t).*(1-t)-1;
clear
load xyz.mat
% i_obstacle=floor(size(x,2)/2);
% obstacle=[x(end),y(end),z(end)];
x=[x;y;z];
%set obstacle
i_obstacle=floor(length(x)/2);
obstacle=x(:,i_obstacle);
OneDimDMPwithVv2(x,1,1,x(:,end),(x(:,end)-x(:,(end-1)))/0.001);
% OneDimDMPwithVv2(y,1,1,y(end),(y(end)-y(end-1))/0.001);
% OneDimDMPwithVv2(z,1,1,z(end),(z(end)-z(end-1))/0.001);