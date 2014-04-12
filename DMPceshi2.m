% 做二维避障测试

clear 
close all
clc
dt=0.001;
t=0:dt:1;
%二维示教轨迹
x=linspace(0,1,length(t));
y=0*ones(1,length(x));
p_orign=[x;y];
dp=p_orign(:,2)-p_orign(:,1);
obs=[0.5,0]';
[p_DMP(:,1:2),gtmp(1:2,:)]=OneDimDMPwithVv3(p_orign,1,1,p_orign(:,end),(p_orign(:,end)-p_orign(:,(end-1)))/0.001);


figure
plot(p_DMP(:,1),p_DMP(:,2),'g*')
hold on ;plot(p_orign(1,:),p_orign(2,:))
axis([0,1,0,1])