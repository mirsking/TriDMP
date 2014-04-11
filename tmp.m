clear all;close all;clc;
DMPcase=1
%% 演示轨迹
syms t;
%%  胡晋师兄的多项式  可演示巨大加速度
% x = 2*t.^2+cos(4*pi*t).*(1-t)-1;
%% 演示轨迹的构造
x=-t.^5+3*t.^4-3*t.^3+t.^2 %演示轨迹的初末位置相同，系统无非线性外力项干涉，整个系统的状态将保持在初始状态
% x=-t.^5+3*t.^4-3*t.^3+t.^2+0.001*t;  %演示镜像效果和过大加速度
xd = diff(x);
xdd = diff(xd);

% time step
dt = 0.001;
% time for demonstration
traj_time=1;
t=0:dt:traj_time;

% 演示轨迹初始化
x=eval(x);
xd=eval(xd);
xdd=eval(xdd);

%% 收敛多项式
% x  = (tpoly(0,1,1001))';
% xd = diff(x)/dt;
% xd = [xd xd(end)];
% xdd = diff(xd)/dt;
% xdd = [xdd xdd(end)];

%% 最后速度匀速多项式

% xd=erf(t/0.3);
% for i=1:length(t)
%     x(i)=sum(xd(1:i))*dt;
% end
% xdd = diff(xd)/dt;
% xdd = [xdd xdd(end)];

%% DMP模型初始化
dmp.D = 15;
dmp.K = dmp.D.^2/4;
dmp.tau=traj_time;  %dmp时间比例因子
dmp.x0=x(1);
dmp.g=x(end);
dmp.A=x(end)-x(1);
dmp.alpha=4;
% 权重函数初始化
n_w=length(t)/10;   %权重函数个数 取100个，数量越多精度越高。最大值为训练数据个数。
dmp.wc=exp(-dmp.alpha/dmp.tau*(0:1/(n_w-1):1)'); %100*1  权重函数中心，注意由于从t映射到s，所以也要像权重函数那样取对数
dmp.wh=(diff(dmp.wc)*0.65).^2;  %99*1  权重函数的宽度是按照中心间距来选择的
dmp.wh=1./[dmp.wh;dmp.wh(end)]; %100*1

% 规范函数
dmp.s = zeros(1,size(x,2));
dmp.s(1) = 1;
for i = 2:length(x),
    dmp.s(i) = dmp.s(i-1)-dmp.alpha*dmp.s(i-1)*dt/dmp.tau;
end
    % dmp.s=Z;
%% w(i)的局部加权学习
% - weighting functions
dmp.psi = exp( -dmp.wh*ones(1,length(x)) .* ( ( ones(length(dmp.wc),1)*dmp.s - dmp.wc*ones(1,length(x)) ).^2 ) );   %100*1001
% original transformation system
switch DMPcase
    case 0
        dmp.f = (dmp.tau.^2*xdd-dmp.K*(dmp.g-x)+dmp.D*dmp.tau*xd)/(dmp.A+1.e-10);   %1*1001
    case 1
        dmp.f = (dmp.tau.^2*xdd-dmp.K*(dmp.g-x)+dmp.D*dmp.tau*xd+dmp.K*dmp.A*dmp.s )/dmp.K;   %1*1001
    case 2
        dmp.vm=dmp.tau*xd(end);
        dmp.gm=x(end)-dmp.vm*traj_time+dmp.vm*t;
        dmp.f = (dmp.tau.^2*xdd-dmp.K*(dmp.gm-x)-dmp.D*(dmp.vm-dmp.tau*xd)+dmp.K*(dmp.gm-dmp.x0).*dmp.s )/dmp.K;   %1*1001
end
% locally weighted linear regression
% wDen = sum( dmp.psi .* ( ones(length(dmp.wc),1)*dmp.s ) ,2);
% wNum = sum( dmp.psi .* ( ones(length(dmp.wc),1)*dmp.f ) ,2);
% dmp.w = wNum./(wDen+1.e-10);
wDen = sum( dmp.psi .* ( ones(length(dmp.wc),1)*(dmp.s.*dmp.s) ) ,2);
wNum = sum( dmp.psi .* ( ones(length(dmp.wc),1)*(dmp.s.*dmp.f) ) ,2);
dmp.w = wNum./(wDen);%+1.e-10
%% 学习后轨迹再现 目标不变
t_run = 0:dt:traj_time;
% dmp参数初始化
dmp.tau=traj_time;  %dmp时间比例因子
dmp.x0=x(1);
% dmp.g=x(end);
dmp.g=x(end);
dmp.A=dmp.g-dmp.x0;
dmp.s1=1;
dmp.x1=x(1);
dmp.v1=dmp.tau*xd(1);
for i=1:length(t_run)
    dmp.psi1 = exp(-dmp.wh.*((dmp.s1-dmp.wc).^2));
    dmp.f1 = sum(dmp.psi1.*dmp.w*dmp.s1)/sum(dmp.psi1+1.e-10);    
    switch DMPcase
        case 0
            dmp.v1_d=(dmp.K*(dmp.g-dmp.x1)-dmp.D*dmp.v1+dmp.A*dmp.f1)/dmp.tau;
        case 1
            dmp.v1_d=(dmp.K*(dmp.g-dmp.x1)-dmp.D*dmp.v1-dmp.K*dmp.A*dmp.s1+dmp.K*dmp.f1)/dmp.tau;
        case 2
            dmp.gm=dmp.g-dmp.vm*traj_time+dmp.vm*t_run(i);
            dmp.v1_d=(dmp.K*(dmp.gm-dmp.x1)+dmp.D*(dmp.vm-dmp.v1)-dmp.K*(dmp.gm-dmp.x0)*dmp.s1+dmp.K*dmp.f1)/dmp.tau;
    end
    dmp.x1_d=dmp.v1/dmp.tau;
    dmp.x1_dd=dmp.v1_d/dmp.tau;
    dmp.s1d = -dmp.alpha*dmp.s1/dmp.tau;
    dmp.s1 = dmp.s1 + dt*dmp.s1d;
    dmp.x1=dmp.x1+dmp.x1_d*dt;
    dmp.v1=dmp.v1+dmp.v1_d*dt;
    
    %每次循环存储数据
    dmp.y(i)=dmp.x1;
    dmp.yd(i)=dmp.x1_d;
    dmp.ydd(i)=dmp.x1_dd;

end

dmp.y0 = dmp.y;
dmp.yd0 = dmp.yd;
dmp.ydd0 = dmp.ydd;
%% 学习后轨迹再现 目标改变
t_run = 0:dt:traj_time;
% dmp参数初始化
dmp.tau=traj_time;  %dmp时间比例因子
dmp.x0=x(1);
% dmp.g=x(end);
dmp.g=0.005;
dmp.vm=0.015;
dmp.A=dmp.g-dmp.x0;
dmp.s1=1;
dmp.x1=x(1);
dmp.v1=dmp.tau*xd(1);
for i=1:length(t_run)
    dmp.psi1 = exp(-dmp.wh.*((dmp.s1-dmp.wc).^2));
    dmp.f1 = sum(dmp.psi1.*dmp.w*dmp.s1)/sum(dmp.psi1+1.e-10);    
    switch DMPcase
        case 0
            dmp.v1_d=(dmp.K*(dmp.g-dmp.x1)-dmp.D*dmp.v1+dmp.A*dmp.f1)/dmp.tau;
        case 1
            dmp.v1_d=(dmp.K*(dmp.g-dmp.x1)-dmp.D*dmp.v1-dmp.K*dmp.A*dmp.s1+dmp.K*dmp.f1)/dmp.tau;
        case 2
            dmp.gm=dmp.g-dmp.vm*traj_time+dmp.vm*t_run(i);
            dmp.v1_d=(dmp.K*(dmp.gm-dmp.x1)+dmp.D*(dmp.vm-dmp.v1)-dmp.K*(dmp.gm-dmp.x0)*dmp.s1+dmp.K*dmp.f1)/dmp.tau;
    end
    dmp.x1_d=dmp.v1/dmp.tau;
    dmp.x1_dd=dmp.v1_d/dmp.tau;
    dmp.s1d = -dmp.alpha*dmp.s1/dmp.tau;
    dmp.s1 = dmp.s1 + dt*dmp.s1d;
    dmp.x1=dmp.x1+dmp.x1_d*dt;
    dmp.v1=dmp.v1+dmp.v1_d*dt;
    
    %每次循环存储数据
    dmp.y(i)=dmp.x1;
    dmp.yd(i)=dmp.x1_d;
    dmp.ydd(i)=dmp.x1_dd;

end

subplot(1,3,1);
plot(t,x,'.g');
hold on
plot(t,dmp.y0,'b');
hold on
plot(t_run,dmp.y,'-.r');
xlabel('t');
ylabel('x')
% axis([0 1 -0.04 0.04]);

subplot(1,3,2);
plot(t,xd,'.g');
hold on
plot(t,dmp.yd0,'b');
hold on
plot(t_run,dmp.yd,'-.r');
xlabel('t');
ylabel('xd')

subplot(1,3,3);
plot(t,xdd,'.g');
hold on
plot(t,dmp.ydd0,'b');
hold on
plot(t_run,dmp.ydd,'-.r');
xlabel('t');
ylabel('xdd')

legend('demonstration','reproduction with same target','reproduction with new target');

