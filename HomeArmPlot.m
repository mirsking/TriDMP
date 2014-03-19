clear 
clc
p1 = mfilename('fullpath');
i=findstr(p1,'\');
p1=p1(1:i(end));
cd E:\MATLABR2013a\toolbox\rvctools
startup_rvc
cd(p1)
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
s='Rz(q1) Ry(q2) Ty(L1) Rx(q3) Tz(L2) Rz(q4) Rx(q5) Tz(L3) Rz(q6) Rx(q7) Tz(L4)';
dh = DHFactor(s);
cmd1 = dh.command('HomeArmR');
HomeArmR = eval(cmd1);

HomeArm(1) = SerialLink(HomeArmR, 'name', 'HomeArmL', 'base', trotz(pi));
HomeArm(2) = SerialLink(HomeArmR, 'name', 'HomeArmR');

plotopt=HomeArm.plot({'noshadow'});
mag=2;
workDims=[-2*reach 2*reach -2*reach 2*reach -2*reach 2*reach]/mag;
axis(workDims)
set(gca, 'Zdir', 'reverse','drawmode', 'fast'); view(137,30);
hold on



load('D:\BCollege\00GraduateDesign\02SourceCode\Panda\simu\EvolvingMind_ZjuPandaArm\bin\Debug\arm_angles.txt');
ql = [arm_angles(:,1),arm_angles(:,4),arm_angles(:,7),arm_angles(:,10),arm_angles(:,13),arm_angles(:,16),arm_angles(:,19)];
qr = [arm_angles(:,22),arm_angles(:,25),arm_angles(:,28),arm_angles(:,31),arm_angles(:,34),arm_angles(:,37),arm_angles(:,40)];
[m,n]=size(ql);




for k=1:10:m
    %HomeArm(1).plot(ql(k,:));
    HomeArm(2).plot(ql(k,:));
    drawnow
end

