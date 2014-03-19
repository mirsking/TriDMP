clear all;close all;clc;

%% 演示轨迹
syms t;
%%  胡晋师兄的多项式  可演示巨大加速度
x = 2*t.^2+cos(4*pi*t).*(1-t)-1;
%% 轨迹的初末位置相同，系统无非线性外力项干涉，整个系统的状态将保持在初始状态
% x=-t.^5+3*t.^4-3*t.^3+t.^2;  %改造后可演示镜像效果
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
dmp.s = exp((-dmp.alpha/dmp.tau).*t); %1*1001  %实测，这种求法与基本求法使dmp.psi差距达到3那么大
% dmp.s=Z;
%% w(i)的局部加权学习
% - weighting functions
dmp.psi = exp( -dmp.wh*ones(1,length(x)) .* ( ( ones(length(dmp.wc),1)*dmp.s - dmp.wc*ones(1,length(x)) ).^2 ) );   %100*1001
% original transformation system
dmp.f = (dmp.tau.^2*xdd-dmp.K*(dmp.g-x)+dmp.D*dmp.tau*xd)/(dmp.A+1.e-10);   %1*1001
% locally weighted linear regression
wDen = sum( dmp.psi .* ( ones(length(dmp.wc),1)*dmp.s ) ,2);
wNum = sum( dmp.psi .* ( ones(length(dmp.wc),1)*dmp.f ) ,2);
dmp.w = wNum./(wDen+1.e-10);
plot(dmp.w)
%% 学习后轨迹再现

t_run = 0:dt:traj_time;
% dmp参数初始化
dmp.tau=traj_time;  %dmp时间比例因子
dmp.x0=x(1);
dmp.g=x(end);
dmp.A=x(end)-x(1);
% dmp.s1=1;
dmp.s1 = exp((-dmp.alpha/dmp.tau).*t_run);
dmp.x1=x(1);
dmp.v1=dmp.tau*xd(1);

for i=1:length(t_run)
    
    dmp.psi1 = exp(-dmp.wh.*((dmp.s1(i)-dmp.wc).^2));
    dmp.f1 = sum(dmp.psi1.*dmp.w*dmp.s1(i))/sum(dmp.psi1+1.e-10);
    dmp.v1_d=(dmp.K*(dmp.g-dmp.x1)-dmp.D*dmp.v1+dmp.A*dmp.f1)/dmp.tau;
    
    dmp.x1_d=dmp.v1/dmp.tau;
    dmp.x1_dd=dmp.v1_d/dmp.tau;
    
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
plot(t_run,dmp.y,'r');

subplot(1,3,2);
plot(t,xd,'.g');
hold on
plot(t_run,dmp.yd,'r');

subplot(1,3,3);
plot(t,xdd,'.g');
hold on
plot(t_run,dmp.ydd,'r');

