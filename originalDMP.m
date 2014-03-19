clear all;close all;clc;

%% ��ʾ�켣
syms t;
%%  ����ʦ�ֵĶ���ʽ  ����ʾ�޴���ٶ�
x = 2*t.^2+cos(4*pi*t).*(1-t)-1;
%% �켣�ĳ�ĩλ����ͬ��ϵͳ�޷�������������棬����ϵͳ��״̬�������ڳ�ʼ״̬
% x=-t.^5+3*t.^4-3*t.^3+t.^2;  %��������ʾ����Ч��
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

%% DMPģ�ͳ�ʼ��
dmp.D = 15;
dmp.K = dmp.D.^2/4;
dmp.tau=traj_time;  %dmpʱ���������
dmp.x0=x(1);
dmp.g=x(end);
dmp.A=x(end)-x(1);
dmp.alpha=4;
% Ȩ�غ�����ʼ��
n_w=length(t)/10;   %Ȩ�غ������� ȡ100��������Խ�ྫ��Խ�ߡ����ֵΪѵ�����ݸ�����
dmp.wc=exp(-dmp.alpha/dmp.tau*(0:1/(n_w-1):1)'); %100*1  Ȩ�غ������ģ�ע�����ڴ�tӳ�䵽s������ҲҪ��Ȩ�غ�������ȡ����
dmp.wh=(diff(dmp.wc)*0.65).^2;  %99*1  Ȩ�غ����Ŀ���ǰ������ļ����ѡ���
dmp.wh=1./[dmp.wh;dmp.wh(end)]; %100*1

% �淶����
dmp.s = exp((-dmp.alpha/dmp.tau).*t); %1*1001  %ʵ�⣬�������������ʹdmp.psi���ﵽ3��ô��
% dmp.s=Z;
%% w(i)�ľֲ���Ȩѧϰ
% - weighting functions
dmp.psi = exp( -dmp.wh*ones(1,length(x)) .* ( ( ones(length(dmp.wc),1)*dmp.s - dmp.wc*ones(1,length(x)) ).^2 ) );   %100*1001
% original transformation system
dmp.f = (dmp.tau.^2*xdd-dmp.K*(dmp.g-x)+dmp.D*dmp.tau*xd)/(dmp.A+1.e-10);   %1*1001
% locally weighted linear regression
wDen = sum( dmp.psi .* ( ones(length(dmp.wc),1)*dmp.s ) ,2);
wNum = sum( dmp.psi .* ( ones(length(dmp.wc),1)*dmp.f ) ,2);
dmp.w = wNum./(wDen+1.e-10);
plot(dmp.w)
%% ѧϰ��켣����

t_run = 0:dt:traj_time;
% dmp������ʼ��
dmp.tau=traj_time;  %dmpʱ���������
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
    
    %ÿ��ѭ���洢����
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

