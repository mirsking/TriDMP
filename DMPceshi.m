clear 
close all
clc
%% ********************************************%
%确定采用师兄的正反解方法，还是工具箱的解法
ToolboxWay=1;%使用工具箱解法

%% ********************************************%
%*****************0 Make Arm Model********************%
%*****************************************************%
%% make robotics toolbox normal
% p1 = mfilename('fullpath');
% i=findstr(p1,'\');
% p1=p1(1:i(end));
% cd E:\MATLABR2013a\toolbox\rvctools
% startup_rvc
% cd(p1)
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
%由于右臂有一些问题，所以目前用的是左臂，但是仿真的时候右臂要好一些，所以用左臂的数据在右臂进行仿真
load('D:\BCollege\00GraduateDesign\02SourceCode\Panda\simu\EvolvingMind_ZjuPandaArm\bin\Debug\arm_angles.txt');
% ql = [arm_angles(:,1),arm_angles(:,4),arm_angles(:,7),arm_angles(:,10),arm_angles(:,13),arm_angles(:,16),arm_angles(:,19)];
qr = [arm_angles(:,22),arm_angles(:,25),arm_angles(:,28),arm_angles(:,31),-arm_angles(:,34),arm_angles(:,37),arm_angles(:,40)];
qr=qr(1:1700,:);%暂时只取准备阶段
length = size(qr,1);
length_d = fix(length/1001);
qr = qr(1:length_d:length_d*1001,:);
%ql = ql(1:500,:);
%% 方法一：直接在关节空间进行DMP
% 彻底失败，一通乱转
% for i=1:7
%     q_m(1,i)=1.2*ql(end,i);
%     q(:,i)=OneDimDMPwithV(ql(:,i)',1,1,ql(end,i),(ql(end,end)-ql(end,end-1))/0.001);
% end

%% 方法二 位姿空间进行DMP
if ToolboxWay==1
    T=HomeArmR.fkine(qr);
else
    for i=1:size(qr,1)
        T(:,:,i)=home_ArmForwardKinematics(qr(i,:),7);
    end
end


p_orign=transl(T);
for i=1:3    
    p_DMP(:,i)=OneDimDMPwithV(p_orign(:,i)',1,1,p_orign(end,i),(p_orign(end,i)-p_orign(end-1,i))/0.001);
end

% q=zeros(7,1,size(ql,1));
% isOK=ones(size(ql,1),1);
% for i=1:size(ql,1)
%     [q(:,:,i),isOK(i,1)]=home_IKNumSolution(transl(T(:,:,i)),t2r(T(:,:,1)), ql(i,:)'); 
% %     [q(:,:,i),isOK(i,1)]=home_IKNumSolution_main(transl(T(:,:,i)),t2r(T(:,:,1)));
% end
%% 仿真
figure
workDims=[-reach reach -reach reach -reach reach];
axis(workDims)
set(gca, 'Zdir', 'reverse','drawmode', 'fast'); view(137,30);
plotopt=HomeArmR.plot({'noshadow'});
hold on
% color=['b','g','r'];
% for i=1:1700:5100
%     plot3(t(i:i+1700-1,1)',t(i:i+1700-1,2)',t(i:i+1700-1,3)',color(fix(i/1700)+1))
% end 
plot3(p_orign(:,1)',p_orign(:,2)',p_orign(:,3)','r')
hold on
plot3(p_DMP(:,1)',p_DMP(:,2)',p_DMP(:,3)','.g')
grid on
hold on
for i=1:size(q,1)
    HomeArmR.plot(q(i,:),plotopt);
end


