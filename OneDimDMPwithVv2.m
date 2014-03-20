function [y,gm]=OneDimDMPwithVv2(x,t_dem,t_run,g_m,v_m)
%% 
%     ������ǣ�1��ʾ�̹켣 ����dem_time x xd xdd
%               2) �滮�켣��ʱ�� traj_time 
%               3) �˶�Ŀ��켣 g_m

%% ��������ʽ
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

%% DMPģ�ͳ�ʼ��
dmp.D = 15;
dmp.K = dmp.D.^2/4;
dmp.tau=t_dem(end)-t_dem(1);  %dmpʱ���������
dmp.x0=x(1);
dmp.g=x(end);
dmp.A=x(end)-x(1);
dmp.alpha=4;
% Ȩ�غ�����ʼ��
n_w=length(t_dem)/10;   %Ȩ�غ������� ȡ100��������Խ�ྫ��Խ�ߡ����ֵΪѵ�����ݸ�����
dmp.wc=exp(-dmp.alpha/dmp.tau*(0:1/(n_w-1):1)'); %100*1  Ȩ�غ������ģ�ע�����ڴ�tӳ�䵽s������ҲҪ��Ȩ�غ�������ȡ����
dmp.wh=(diff(dmp.wc)*0.65).^2;  %99*1  Ȩ�غ����Ŀ���ǰ������ļ����ѡ���
dmp.wh=1./[dmp.wh;dmp.wh(end)]; %100*1

% �淶����
% dmp.s = exp((-dmp.alpha/dmp.tau).*t_dem); %1*1001  %ʵ�⣬�������������ʹdmp.psi���ﵽ3��ô��
dmp.s = zeros(size(x));
dmp.s(1) = 1;
for i = 2:length(x),
    dmp.s(i) = dmp.s(i-1)-dmp.alpha*dmp.s(i-1)*dt/dmp.tau;
end
%% w(i)�ľֲ���Ȩѧϰ
% - weighting functions
dmp.psi = exp( -dmp.wh*ones(1,length(x)) .* ( ( ones(length(dmp.wc),1)*dmp.s - dmp.wc*ones(1,length(x)) ).^2 ) );   %100*1001
dmp.vm=dmp.tau*xd(end);
dmp.gm=x(end)-dmp.vm*dem_time+dmp.vm*t_dem;
dmp.f = (dmp.tau.^2*xdd-dmp.K*(dmp.gm-x)-dmp.D*(dmp.vm-dmp.tau*xd)+dmp.K*(dmp.gm-dmp.x0).*dmp.s )/dmp.K;   %1*1001

% locally weighted linear regression
wDen = sum( dmp.psi ,2);
wNum = sum( dmp.psi .* ( ones(length(dmp.wc),1)*dmp.f ) ,2);
dmp.w = wNum./(wDen);%+1.e-10
%% ѧϰ��켣����
% dmp������ʼ��
traj_time=t_run;
t_run=0:dt:t_run;
dmp.tau=t_run(end)-t_run(1);  %dmpʱ���������
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
    
    %ÿ��ѭ���洢����
    dmp.y(i)=dmp.x1;
    dmp.yd(i)=dmp.x1_d;
    dmp.ydd(i)=dmp.x1_dd;

end
y=[dmp.y'];
gm=dmp.tmp;
% % y=[dmp.y',dmp.yd',dmp.ydd'];
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

