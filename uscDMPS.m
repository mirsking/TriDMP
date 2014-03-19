% This file contains a sample implementation of motor primitives
% from USC model

clear all;close all;clc;
syms t;
x = 2*t.^2+cos(4*pi*t).*(1-t)-1;
xd = diff(x);
xdd = diff(xd);



% time step
dt = 0.001;

% time for demonstration
movement_time = 1;
t = 0:dt:movement_time;
tau = 1./movement_time;

% —› æπÏº£≥ı ºªØ
demo=eval(x);
demo_d=eval(xd);
demo_dd=eval(xdd);

% number of weights
n_w = length(demo)/10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initialize motor primitive constants
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% critically damped
hmp.alpha_g = 15;
hmp.beta_g = hmp.alpha_g/4;
hmp.alpha_z = 4;
% initialize centers equispaced in time
hmp.c = exp(-hmp.alpha_z*(0:1/(n_w-1):1)');
hmp.h = (diff(hmp.c)*0.65).^2;
hmp.h = 1./[hmp.h;hmp.h(end)];

%%%%%%%%%%%%%%%%%%%%%%%
% imitate demonstration
%%%%%%%%%%%%%%%%%%%%%%%
% initial position
hmp.x1_0 = demo(1);
% final position and velocity
hmp.target_f = demo(end);
%hmp.target_d = demo_d(end);
% amplitude
hmp.A = demo(end)-demo(1);

% moving target
% target = ones(size(demo))*(hmp.target_f-movement_time*hmp.target_d);
% target = target + t*hmp.target_d;

% compute the transformation function values:
% - canonical system
Z = zeros(size(demo));
Z(1) = 1;
for i = 2:length(demo),
    Z(i) = Z(i-1)-hmp.alpha_z*Z(i-1)*tau*dt;
end

% - weighting functions
PSI = exp( -hmp.h*ones(1,length(demo)) .* ( ( ones(length(hmp.c),1)*Z - hmp.c*ones(1,length(demo)) ).^2 ) );
% - transformation function
%fdemo = (demo_dd/tau^2-hmp.alpha_g*(1-Z).*(hmp.beta_g*(target-demo)+(hmp.target_d-demo_d)/tau))/hmp.A;
fdemo = (demo_dd/tau^2-hmp.alpha_g*(hmp.beta_g*(hmp.target_f-demo)-(demo_d)/tau)+hmp.alpha_g*hmp.beta_g*hmp.A *Z)/(hmp.alpha_g*hmp.beta_g);

%fdemo = (demo_dd/tau^2-hmp.alpha_g*(hmp.beta_g*(hmp.target_f-demo)-(demo_d)/tau))/hmp.A;

% locally weighted linear regression
wDnom = sum( PSI .* ( ones(length(hmp.c),1)*( Z.^2 ) ) ,2);
wNom = sum( PSI .* ( ones(length(hmp.c),1)*( Z.*fdemo ) ) ,2);
hmp.w = wNom./(wDnom+1.e-10);


%%%%%%%%%%%%%%%%%%%%%
% run motor primitive
%%%%%%%%%%%%%%%%%%%%%
% time for motor primitves
t_run = 0:dt:1.5*movement_time;
%t_run = 0:dt:movement_time;
% initialize motor primitive
% - canonical system
hmp.z = 1;
% - initial postion and velocity
hmp.x1 = 0;
hmp.x2 = demo_d(1)*movement_time;
% - initial postion of the moving target
%hmp.target = hmp.target_f-movement_time*hmp.target_d;
hmp.target = hmp.target_f;

y = demo(1);
yd = demo_d(1);
ydd = demo_dd(1);

for i = 1:length(t_run)
    % move target
   % hmp.target = hmp.target + hmp.target_d*dt;
    
    % calculate weighting & transformation function
    hmp.psi = exp(-hmp.h.*((hmp.z-hmp.c).^2));
    f = sum(hmp.psi.*hmp.w*hmp.z)/sum(hmp.psi+1.e-10);
    
    % use simplectic Euler integration
    hmp.x2_d = (hmp.alpha_g*(hmp.beta_g*(hmp.target-hmp.x1)-hmp.x2) + hmp.alpha_g*hmp.beta_g*f-hmp.alpha_g*hmp.beta_g*hmp.A*hmp.z)*tau;
    hmp.x1_d = hmp.x2*tau;
    hmp.zd = -hmp.alpha_z*hmp.z*tau;
    hmp.x1_dd = hmp.x2_d*tau;
    
    hmp.x2 = hmp.x2 + dt*hmp.x2_d;
    hmp.x1 = hmp.x1 + dt*hmp.x1_d;
    hmp.z = hmp.z + dt*hmp.zd;
    % store values
    y(i) = hmp.x1;
    yd(i) = hmp.x1_d;
    ydd(i) = hmp.x1_dd;
end;

subplot(1,3,1);
plot(t_run,y,'k');
hold 'on';

subplot(1,3,2);
plot(t_run,yd,'k');
hold 'on';

subplot(1,3,3);
plot(t_run,ydd,'k');
hold 'on';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% run generalized motor primitive
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initialize motor primitive
% - canonical system
hmp.z = 1;
% - initial postion and velocity
hmp.x1 = 0;
hmp.x2 = demo_d(1)*movement_time;
% - new final velocity
%hmp.target_d = 2*hmp.target_d;
% - initial postion of the moving target
hmp.target = hmp.target_f+0.2;

y = demo(1);
yd = demo_d(1);
ydd = demo_dd(1);

for i = 1:length(t_run)
    % move target
    %hmp.target = hmp.target + hmp.target_d*dt;
    
    % calculate weighting & transformation function
    hmp.psi = exp(-hmp.h.*((hmp.z-hmp.c).^2));
    f = sum(hmp.psi.*hmp.w*hmp.z)/sum(hmp.psi+1.e-10);
    
    % use simplectic Euler integration
hmp.x2_d = (hmp.alpha_g*(hmp.beta_g*(hmp.target-hmp.x1)-hmp.x2) + hmp.alpha_g*hmp.beta_g*f-hmp.alpha_g*hmp.beta_g*hmp.A*hmp.z)*tau;
    hmp.x1_d = hmp.x2*tau;
    hmp.zd = -hmp.alpha_z*hmp.z*tau;
    hmp.x1_dd = hmp.x2_d*tau;
    
    hmp.x2 = hmp.x2 + dt*hmp.x2_d;
    hmp.x1 = hmp.x1 + dt*hmp.x1_d;
    hmp.z = hmp.z + dt*hmp.zd;
    % store values
    y(i) = hmp.x1;
    yd(i) = hmp.x1_d;
    ydd(i) = hmp.x1_dd;
end;

subplot(1,3,1);
plot(t_run,y,'r');
plot(t,demo,'b');
axis([0 1.5 -7.5 7.5]);
ylabel('q');
xlabel('time');
title('positions','FontAngle','italic','FontWeight','bold');

subplot(1,3,2);
plot(t_run,yd,'r');
plot(t,demo_d,'b');
ylabel('qd');
xlabel('time');
title('velocities','FontAngle','italic','FontWeight','bold');

subplot(1,3,3);
plot(t_run,ydd,'r');
plot(t,demo_dd,'b');
ylabel('qdd');
xlabel('time');
title('accelerations','FontAngle','italic','FontWeight','bold');

legend('reproduction','generalization','demonstration','Location','SouthEast');