clear 
clc
%定义的状态参数
qz = zeros(1,7);
qs=[0,0,pi,0,0,0,0];
%手臂参数
L1=0.142;
L2=0.281;
L3=0.211;
L4=0.200;
reach=L1+L2+L3+L4;
%反解DH法
s='Rz(q1) Ry(q2) Ty(L1) Rx(q3) Tz(L2) Rz(q4) Rx(q5) Tz(L3) Rz(q6) Rx(q7) Tz(L4)'
dh = DHFactor(s);
cmd1 = dh.command('HomeArmR');
HomeArmR = eval(cmd1);
cmd2 = dh.command('HomeArmL');
HomeArmL = eval(cmd2);
HomeArmL.base=trotz(pi);

mag=1.5;
workDims=[-1.5*reach 1.5*reach -1.5*reach 1.5*reach -reach 1.5*reach]/mag;
axis(workDims)
plotopt = HomeArmR.plot({'noraise', 'noshadow','nowrist', 'nojaxes'});
set(gca, 'Zdir', 'reverse','drawmode', 'fast'); view(137,30);
hold on

load arm_angles.txt
tmp=arm_angles(100:500:end,:);
arm_angles=0;
arm_angles=tmp;
ql = [arm_angles(:,1),arm_angles(:,4),arm_angles(:,7),arm_angles(:,10),arm_angles(:,13),arm_angles(:,16),arm_angles(:,19)];
qr = [arm_angles(:,22),arm_angles(:,25),arm_angles(:,28),arm_angles(:,31),-arm_angles(:,34),arm_angles(:,37),arm_angles(:,40)];

%% 绘图仿真

% [m,n]=size(ql);
%  for i=1:10:m
%     HomeArmL.plot(ql(i,:),plotopt)
%     hold on
%     HomeArmR.plot(qr(i,:),plotopt)
%  end
 
%% 获取坐标数据
traj=HomeArmR.fkine(qr);
HomeArmR.tool=traj(:,:,1);
Q=HomeArmR.ikine(traj);
rpy = tr2rpy(traj);
%提取移动坐标信息
traj=traj(1:3,4,:);
traj=permute(traj,[3,2,1]);
% x=traj(:,:,1);
% y=traj(:,:,2);
% z=traj(:,:,3);
% 
% plot3(x,y,z)
% grid on
% hold on
% plot3(x(797),y(797),z(797),'*g')

xx=traj(:,:,1);
yy=traj(:,:,2);
zz=traj(:,:,3);
plot3(xx,yy,zz)
grid on

save('TriDimData.mat','xx','yy','zz','rpy');


