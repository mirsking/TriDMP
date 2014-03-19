clear all;close all;clc;

%% 演示轨迹
syms t;
%%  胡晋师兄的多项式  可演示巨大加速度
% 巨大加速的问题在改进模型中仍未解决
 x = 2*t.^2+cos(4*pi*t).*(1-t)-1;
%% 轨迹的初末位置相同，系统无非线性外力项干涉，整个系统的状态将保持在初始状态
% 改进模型中这个问题已经解决
% x=-t.^5+3*t.^4-3*t.^3+t.^2;  %改造后可演示镜像效果 镜像效果在改进模型中已经解决
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
%将示教轨迹存入文本文件中
f_handle=fopen('..\..\VC++\modifiedDMP\Data\dem.txt','w');
for i=1:length(x)
    fprintf(f_handle,'%g\t%g\t%g\t\n',x(i),xd(i),xdd(i));
%     fprintf(f_handle,'%g\t\n',x(i));%只存储位置信息
end
fclose(f_handle);
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

% %% 对x xd xdd 的扩充，添加匀速运动阶段
% traj_time_vm=0.5;
% t_vm=dt:dt:traj_time_vm;
% dmp.vm=2;
% dmp.gm0=x(end);
% dmp.gm=dmp.gm0+dmp.vm*t_vm;
% dmp.gm=[ones(1,length(x)) dmp.gm];
% t=[t,t_vm+t(end)];
% x = [x dmp.gm(length(x)+1:end)];
% xd = [xd ones(1,length(t_vm))*xd(end)];
% xdd = [xdd zeros(1,length(t_vm))];
% traj_time= traj_time+traj_time_vm;

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
%% w(i)的局部加权学习
% - weighting functions
dmp.psi = exp( -dmp.wh*ones(1,length(x)) .* ( ( ones(length(dmp.wc),1)*dmp.s - dmp.wc*ones(1,length(x)) ).^2 ) );   %100*1001
% modified transformation system1
dmp.f = (dmp.tau.^2*xdd-dmp.K*(dmp.g-x)+dmp.D*dmp.tau*xd+dmp.K*dmp.A*dmp.s )/dmp.K;   %1*1001
% locally weighted linear regression
wDen = sum( dmp.psi .* ( ones(length(dmp.wc),1)*dmp.s ) ,2);
wNum = sum( dmp.psi .* ( ones(length(dmp.wc),1)*dmp.f ) ,2);
dmp.w = wNum./(wDen+1.e-10);
plot(dmp.w)
%% 学习后轨迹再现
 traj_time=1.5*traj_time;%调整时间
t_run = 0:dt:traj_time;
% dmp参数初始化
dmp.tau=traj_time;  %dmp时间比例因子
dmp.x0=x(1);
dmp.g=x(end);%调整目标
dmp.A=x(end)-x(1);
% dmp.s1=1;
dmp.s1 = exp((-dmp.alpha/dmp.tau).*t_run);
dmp.x1=x(1);
dmp.v1=dmp.tau*xd(1);

for i=1:length(t_run)
%     if i<2/3*length(t_run)
       % dmp.g=dmp.g+0.001;   
%     end
    tmp(i) = dmp.g;
    dmp.psi1 = exp(-dmp.wh.*((dmp.s1(i)-dmp.wc).^2));
    dmp.f1 = sum(dmp.psi1.*dmp.w*dmp.s1(i))/sum(dmp.psi1+1.e-10);
    dmp.v1_d=(dmp.K*(dmp.g-dmp.x1)-dmp.D*dmp.v1-dmp.K*dmp.A*dmp.s1(i)+dmp.K*dmp.f1)/dmp.tau;
    
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
plot(t_run,tmp,'k')
hold on
plot(t_run,dmp.y,'r');
hold on
legend('demonstration','target','gerneralization');

subplot(1,3,2);
plot(t,xd,'.g');
hold on
plot(t_run,dmp.yd,'r');

subplot(1,3,3);
plot(t,xdd,'.g');
hold on
plot(t_run,dmp.ydd,'r');



