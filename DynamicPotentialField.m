close all;clear all;clc
x=-1:0.01:1;
y=-1:0.01:1;
%障碍点在(0,0)
%向量x=(x,y)
px=sqrt(x.^2+y.^2);
beta = 2;
lambda = 1;
v=[-1,-2]';
costheta=((v*ones(1,size(x,2)))*[x',y'])./(px*sqrt(5));
z=(costheta.^2)./px;
mesh(x,y,z,z/10)