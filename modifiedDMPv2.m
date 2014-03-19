clear all;close all;clc;

%% ��ʾ�켣
syms t;
%%  ����ʦ�ֵĶ���ʽ  ����ʾ�޴���ٶ�
% �޴���ٵ������ڸĽ�ģ������δ���
 x = 2*t.^2+cos(4*pi*t).*(1-t)-1;
%% �켣�ĳ�ĩλ����ͬ��ϵͳ�޷�������������棬����ϵͳ��״̬�������ڳ�ʼ״̬
% �Ľ�ģ������������Ѿ����
% x=-t.^5+3*t.^4-3*t.^3+t.^2;  %��������ʾ����Ч�� ����Ч���ڸĽ�ģ�����Ѿ����
xd = diff(x);
xdd = diff(xd);

% time step
dt = 0.001;
% time for demonstration
traj_time=1;
t=0:dt:traj_time;

% ��ʾ�켣��ʼ��
x=eval(x);
xd=eval(xd);
xdd=eval(xdd);
%% ��������ʽ
% x  = (tpoly(0,1,1001))';
% xd = diff(x)/dt;
% xd = [xd xd(end)];
% xdd = diff(xd)/dt;
% xdd = [xdd xdd(end)];

%% ����ٶ����ٶ���ʽ
% xd=erf(t/0.3);
% for i=1:length(t)
%     x(i)=sum(xd(1:i))*dt;
% end
% xdd = diff(xd)/dt;
% xdd = [xdd xdd(end)];

%% ��x xd xdd �����䣬��������˶��׶�
% traj_time_vm=0.5;
% t_vm=dt:dt:traj_time_vm;
% dmp.vm=0;
% dmp.gm0=x(end);
% dmp.gm=dmp.gm0;%+dmp.vm*t_vm;
% dmp.gm=[x(end)*ones(1,length(x)) dmp.gm];
% t=[t,t_vm+t(end)];
% x = [x dmp.gm(length(x)+1:end)];
% xd = [xd ones(1,length(t_vm))*xd(end)];
% xdd = [xdd zeros(1,length(t_vm))];
% traj_time= traj_time+traj_time_vm;

%% DMPģ�ͳ�ʼ��
dmp.D = 15;
dmp.K = dmp.D.^2/4;
dmp.tau=traj_time;  %dmpʱ���������
dmp.x0=x(1);
dmp.g=x(end);
dmp.A=x(end)-x(1);
dmp.alpha=4;
% Ȩ�غ�����ʼ��
n_w=length(t)/1;   %Ȩ�غ������� ȡ100��������Խ�ྫ��Խ�ߡ����ֵΪѵ�����ݸ�����
dmp.wc=exp(-dmp.alpha/dmp.tau*(0:1/(n_w-1):1)'); %100*1  Ȩ�غ������ģ�ע�����ڴ�tӳ�䵽s������ҲҪ��Ȩ�غ�������ȡ����
dmp.wh=(diff(dmp.wc)*0.65).^2;  %99*1  Ȩ�غ����Ŀ���ǰ������ļ����ѡ���
dmp.wh=1./[dmp.wh;dmp.wh(end)]; %100*1


% �淶����
% dmp.s = exp((-dmp.alpha/dmp.tau).*t); %1*1001  %ʵ�⣬�������������ʹdmp.psi���ﵽ3��ô��
dmp.s = zeros(size(x));
dmp.s(1) = 1;
for i = 2:length(x),
    dmp.s(i) = dmp.s(i-1)-dmp.alpha*dmp.s(i-1)*dmp.tau*dt;
end
%% w(i)�ľֲ���Ȩѧϰ
% - weighting functions
dmp.psi = exp( -dmp.wh*ones(1,length(x)) .* ( ( ones(length(dmp.wc),1)*dmp.s - dmp.wc*ones(1,length(x)) ).^2 ) );   %100*1001
% modified transformation system1
% dmp.f = (dmp.tau.^2*xdd-dmp.K*(dmp.g-x)+dmp.D*dmp.tau*xd+dmp.K*dmp.A*dmp.s )/dmp.K;   %1*1001

dmp.vm=dmp.tau*xd(end);
dmp.gm=x(end)-dmp.vm*traj_time+dmp.vm*t;
dmp.f = (dmp.tau.^2*xdd-dmp.K*(dmp.gm-x)-dmp.D*(dmp.vm-dmp.tau*xd)+dmp.K*(dmp.gm-dmp.x0).*dmp.s )/dmp.K;   %1*1001
% fdemo = (demo_dd/tau^2-hmp.alpha_g*(1-Z).*(hmp.beta_g*(target-demo)+(hmp.target_d-demo_d)/tau))/hmp.A;
% locally weighted linear regression
wDen = sum( dmp.psi .* ( ones(length(dmp.wc),1)*(dmp.s) ) ,2);
wNum = sum( dmp.psi .* ( ones(length(dmp.wc),1)*(dmp.f) ) ,2);
dmp.w = wNum./(wDen+1.e-10);
%% ѧϰ��켣����
t_run = 0:dt:traj_time;
% dmp������ʼ��
dmp.tau=1;  %dmpʱ���������
dmp.x0=x(1);
dmp.g=1.5*x(end);
dmp.vm=3.5;
dmp.A=x(end)-x(1);
% dmp.s1=1;
dmp.s1=1;
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


subplot(1,3,1);
plot(t,x,'.g');
hold on
plot(t_run,dmp.tmp,'k')
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

