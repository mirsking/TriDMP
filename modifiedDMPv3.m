clear all;close all;clc;

%% ��ʾ�켣
syms t;
%%  ����ʦ�ֵĶ���ʽ  ����ʾ�޴���ٶ�
% �޴���ٵ������ڸĽ�3ģ���еõ�����
 x = 2*t.^2+cos(4*pi*t).*(1-t)-1;
%% �켣�ĳ�ĩλ����ͬ��ϵͳ�޷�������������棬����ϵͳ��״̬�������ڳ�ʼ״̬
% �Ľ�ģ������������Ѿ����
%  x=-t.^5+3*t.^4-3*t.^3+t.^2;  %��������ʾ����Ч�� ����Ч���ڸĽ�ģ�����Ѿ����
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
%% w(i)�ľֲ���Ȩѧϰ
% - weighting functions
dmp.psi = exp( -dmp.wh*ones(1,length(x)) .* ( ( ones(length(dmp.wc),1)*dmp.s - dmp.wc*ones(1,length(x)) ).^2 ) );   %100*1001
% sigma weighting functions  ��������Ĺ��죬ֱ��Ӱ���ˣ����Ŀ��ʱ��ļ��ٶ�
wg=0.5+0.5*erf((t-0.5*dmp.tau)/sqrt(2)/0.2/dmp.tau);
i=min(find(wg>0.99));
dd=(wg(end)-wg(i))/(t(end)-t(i));
for k=i:1:length(wg)
    wg(k)=wg(i)+dd*(t(k)-t(i));
end
wg(end)=wg(end-1);
% modified transformation system v3 
dmp.f = (dmp.tau.^2*xdd-wg.*(dmp.K*(dmp.g-x))+dmp.D*dmp.tau*xd)./(1-wg)-dmp.x0+x; %1*1001
% locally weighted linear regression
wDen = sum( dmp.psi ,2);
wNum = sum( dmp.psi .* ( ones(length(dmp.wc),1)*dmp.f ) ,2);
dmp.w = wNum./(wDen);%+1.e-10
%% ѧϰ��켣����
traj_time=1.5*traj_time;
t_run = 0:dt:traj_time;
% dmp������ʼ��
dmp.tau=traj_time;  %dmpʱ���������
dmp.x0=x(1);
dmp.g=x(end)+1;
dmp.A=x(end)-x(1);
% dmp.s1=1;
dmp.s1 = exp((-dmp.alpha/dmp.tau).*t_run);
dmp.x1=x(1);
dmp.v1=dmp.tau*xd(1);

%Ȩ��
wg=0.5+0.5*erf((t_run-0.5*dmp.tau)/sqrt(2)/0.2/dmp.tau);
i=min(find(wg>0.99));
dd=(wg(end)-wg(i))/(t_run(end)-t_run(i));
for k=i:1:length(wg)
    wg(k)=wg(i)+dd*(t_run(k)-t_run(i));
end
wg(end)=wg(end)-1.0e-10;

for i=1:length(t_run)
    if i<1/3*length(t_run)
        dmp.g=dmp.g+0.001;   
    end
    tmp(i) = dmp.g;
    dmp.psi1 = exp(-dmp.wh.*((dmp.s1(i)-dmp.wc).^2));
    dmp.f1 = sum(dmp.psi1.*dmp.w)/sum(dmp.psi1);
%     dmp.v1_d=(...
%     (1-wg(i))*(dmp.f1+dmp.x0-dmp.x1)...
%     +wg(i)*(dmp.K*(dmp.g-dmp.x1))...
%     -dmp.D*dmp.v1)/dmp.tau;
    dmp.v1_d = (dmp.K*(dmp.g-dmp.x1))*wg(i) + (dmp.f1+dmp.x0-dmp.x1)*(1-wg(i))-dmp.D*dmp.v1;
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
plot(t_run,tmp,'k')
hold on
plot(t_run,dmp.y,'r');
axis([0 1.5 -3 5]);

subplot(1,3,2);
plot(t,xd,'.g');
hold on
plot(t_run,dmp.yd,'r');
axis([0 1.5 -10 10]);

subplot(1,3,3);
plot(t,xdd,'.g');
hold on
plot(t_run,dmp.ydd,'r');
axis([0 1.5 -180 180]);

