function [y,gm]=OneDimDMPwithV(x,t_dem,t_run,g_m,v_m)
%% 
%     输入的是：1）示教轨迹 包括dem_time x xd xdd
%               2) 规划轨迹的时间 traj_time 
%               3) 运动目标轨迹 g_m

%% 收敛多项式
% x  = (tpoly(0,1,1001))';
% xd = diff(x)/dt;
% xd = [xd xd(end)];
% xdd = diff(xd)/dt;
% xdd = [xdd xdd(end)];
dt=0.001;
xd = diff(x);
xd =[xd, xd(end)]/dt;
xdd = diff(xd);
xdd=[xdd,xdd(end)]/dt;
dem_time=t_dem;
t_dem=0:dt:t_dem;

%% DMP模型初始化
dmp.D = 15;
dmp.K = dmp.D.^2/4;
dmp.tau=t_dem(end)-t_dem(1);  %dmp时间比例因子
dmp.x0=x(1);
dmp.g=x(end);
dmp.A=x(end)-x(1);
dmp.alpha=4;
% 权重函数初始化
n_w=length(t_dem)/10;   %权重函数个数 取100个，数量越多精度越高。最大值为训练数据个数。
dmp.wc=exp(-dmp.alpha/dmp.tau*(0:1/(n_w-1):1)'); %100*1  权重函数中心，注意由于从t映射到s，所以也要像权重函数那样取对数
dmp.wh=(diff(dmp.wc)*0.65).^2;  %99*1  权重函数的宽度是按照中心间距来选择的
dmp.wh=1./[dmp.wh;dmp.wh(end)]; %100*1

% 规范函数
% dmp.s = exp((-dmp.alpha/dmp.tau).*t_dem); %1*1001  %实测，这种求法与基本求法使dmp.psi差距达到3那么大
dmp.s = zeros(size(x));
dmp.s(1) = 1;
for i = 2:length(x),
    dmp.s(i) = dmp.s(i-1)-dmp.alpha*dmp.s(i-1)*dt/dmp.tau;
end
%% w(i)的局部加权学习
% - weighting functions
dmp.psi = exp( -dmp.wh*ones(1,length(x)) .* ( ( ones(length(dmp.wc),1)*dmp.s - dmp.wc*ones(1,length(x)) ).^2 ) );   %100*1001
dmp.vm=dmp.tau*xd(end);
dmp.gm=x(end)-dmp.vm*dem_time+dmp.vm*t_dem;
dmp.f = (dmp.tau.^2*xdd-dmp.K*(dmp.gm-x)-dmp.D*(dmp.vm-dmp.tau*xd)+dmp.K*(dmp.gm-dmp.x0).*dmp.s )/dmp.K;   %1*1001

% locally weighted linear regression
% wDen = sum( dmp.psi ,2);
% wNum = sum( dmp.psi .* ( ones(length(dmp.wc),1)*dmp.f ) ,2);
wDen = sum( dmp.psi .* ( ones(length(dmp.wc),1)*(dmp.s.*dmp.s) ) ,2);
wNum = sum( dmp.psi .* ( ones(length(dmp.wc),1)*(dmp.s.*dmp.f) ) ,2);
dmp.w = wNum./(wDen);%+1.e-10
%% 学习后轨迹再现
% dmp参数初始化
traj_time=t_run;
t_run=0:dt:t_run;
dmp.tau=t_run(end)-t_run(1);  %dmp时间比例因子
dmp.x0=x(1);
dmp.g=g_m;
dmp.vm=v_m;
dmp.A=dmp.g-x(1);
dmp.s1=1;
% dmp.s1 = exp((-dmp.alpha/dmp.tau).*t_run);
dmp.x1=x(1);
dmp.v1=dmp.tau*xd(1);
for i=1:length(t_run)
    
    dmp.gm=dmp.g-dmp.vm*traj_time+dmp.vm*t_run(i);
    dmp.tmp(i)=dmp.gm;
    dmp.psi1 = exp(-dmp.wh.*((dmp.s1-dmp.wc).^2));
    dmp.f1 = sum(dmp.psi1.*dmp.w*dmp.s1)/sum(dmp.psi1+1.e-10);
    dmp.v1_d=(dmp.K*(dmp.gm-dmp.x1)+dmp.D*(dmp.vm-dmp.v1)-dmp.K*(dmp.gm-dmp.x0)*dmp.s1+dmp.K*dmp.f1)/dmp.tau;
    
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
y=[dmp.y'];
gm=dmp.tmp;
% % y=[dmp.y',dmp.yd',dmp.ydd'];
if nargout==0
    figure;
    subplot(1,3,1);
    plot(t_dem,x,'.g');
    hold on
    plot(t_run,dmp.tmp,'k')
    hold on
    plot(t_run,dmp.y,'r');

    subplot(1,3,2);
    plot(t_dem,xd,'.g');
    hold on
    plot(t_run,dmp.yd,'r');

    subplot(1,3,3);
    plot(t_dem,xdd,'.g');
    hold on
    plot(t_run,dmp.ydd,'r');
end
end

