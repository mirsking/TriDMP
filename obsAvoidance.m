clear all;close all;clc

dt=0.1;
t=0:dt:1;

%原轨迹
x=linspace(0,1,length(t));
y=0*ones(1,length(x));
p=[x;y];
obs=[0.5,0]';
plot(p(1,:),p(2,:),'g*')
hold on
plot(obs(1),obs(2),'bo')

v=[diff(p,1,2)/dt];
v=[v,v(:,end)];

%计算障碍势场，也即加速度
for i=1:length(x)
    F=obstacle(p(:,i),v(:,i));
    v(:,i+1)=v(:,i)+F'*dt;
    p(:,i+1)=p(:,i)+v(:,i)*dt;
end

figure
plot(p(1,:),p(2,:),'g*')
hold on
plot(obs(1),obs(2),'bo')

t_new=[t,t(end)+dt];
plot(v(1,:),v(2,:))