%% ********************************************%
%*****************0 Make Arm Model********************%
%*****************************************************%
clear 
close all
clc
%make robotics toolbox normal
% p1 = mfilename('fullpath');
% i=findstr(p1,'\');
% p1=p1(1:i(end));
% cd E:\MATLABR2013a\toolbox\rvctools
% startup_rvc
% cd(p1)
%�ֱ۲���
L1=0.142;
L2=0.281;
L3=0.211;
L4=0.200;
reach=L1+L2+L3+L4;
%����DH��
s='Rz(q1) Ry(q2) Ty(L1) Rx(q3) Tz(L2) Rz(q4) Rx(q5) Tz(L3) Rz(q6) Rx(q7) Tz(L4)';
dh = DHFactor(s);
cmd1 = dh.command('HomeArmR');
HomeArmR = eval(cmd1);
%�����ұ���һЩ���⣬����Ŀǰ�õ�����ۣ����Ƿ����ʱ���ұ�Ҫ��һЩ����������۵��������ұ۽��з���
load('D:\BCollege\00GraduateDesign\02SourceCode\Panda\simu\EvolvingMind_ZjuPandaArm\bin\Debug\arm_angles.txt');
ql = [arm_angles(:,1),arm_angles(:,4),arm_angles(:,7),arm_angles(:,10),arm_angles(:,13),arm_angles(:,16),arm_angles(:,19)];
length = size(ql,1);
length_d = fix(length/1001);
ql = ql(1:length_d:length_d*1001,:);
%% ����һ��ֱ���ڹؽڿռ����DMP
% ����ʧ�ܣ�һͨ��ת
% for i=1:7
%     q_m(1,i)=1.2*ql(end,i);
%     q(:,i)=OneDimDMPwithV(ql(:,i)',1,1,ql(end,i),(ql(end,end)-ql(end,end-1))/0.001);
% end

%% ������ λ�˿ռ����DMP
T=HomeArmR.fkine(ql(500:end,:));
qq=HomeArmR.ikine(T);
%% ����
figure
workDims=[-reach reach -reach reach -reach reach];
axis(workDims)
set(gca, 'Zdir', 'reverse','drawmode', 'fast'); view(137,30);
plotopt=HomeArmR.plot({'noshadow'});
hold on
[R,t]=tr2rt(HomeArmR.fkine(q_m));
plot3(t(1),t(2),t(3),'g*');
for i=1:size(q,1)
    HomeArmR.plot(q(i,:),plotopt);
end



