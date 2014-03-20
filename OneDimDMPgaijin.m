clear
close all
clc

dt =0.001;
t=0:dt:1;
x = 2*t.^2+cos(4*pi*t).*(1-t)-1;
clear
load x.mat
OneDimDMPwithVv2(x,1,1,x(end),(x(end)-x(end-1))/0.001);