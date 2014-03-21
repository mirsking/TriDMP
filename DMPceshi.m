clear 
close all
clc
%% ********************************************%
%ȷ������ʦ�ֵ������ⷽ�������ǹ�����Ľⷨ
ToolboxWay=1;%ʹ�ù�����ⷨ
% ToolboxWay=0;%ʹ��ʦ�ֵĽⷨ
%�Ƿ����̬����DMP
% RotFlag=1;%����̬���е���
RotFlag=0;%������̬����DMP
%�Ƿ񱣴�Ϊgif
SaveGif=1;%����ΪGIF
% SaveGif=0;%������ΪGIF
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
% ql = [arm_angles(:,1),arm_angles(:,4),arm_angles(:,7),arm_angles(:,10),arm_angles(:,13),arm_angles(:,16),arm_angles(:,19)];
qr = [arm_angles(:,22),arm_angles(:,25),arm_angles(:,28),arm_angles(:,31),-arm_angles(:,34),arm_angles(:,37),arm_angles(:,40)];
qr=qr(1:1700,:);%��ʱֻȡ׼���׶�
length = size(qr,1);
length_d = fix(length/1001);
qr = qr(1:length_d:length_d*1001,:);
%ql = ql(1:500,:);
%% ����һ��ֱ���ڹؽڿռ����DMP
% ����ʧ�ܣ�һͨ��ת
% for i=1:7
%     q_m(1,i)=1.2*ql(end,i);
%     q(:,i)=OneDimDMPwithV(ql(:,i)',1,1,ql(end,i),(ql(end,end)-ql(end,end-1))/0.001);
% end

%% ������ λ�˿ռ����DMP
if ToolboxWay==1
    T=HomeArmR.fkine(qr);
else
    for i=1:size(qr,1)
        T(:,:,i)=home_ArmForwardKinematics(qr(i,:),7);
    end
end
%λ��DMP
p_orign=transl(T)';
i_obstacle=fix(size(p_orign,2)/2);
% i_obstacle=1;
obstacle=p_orign(:,i_obstacle);
% for i=1:3    
%     [p_DMP(:,i),gtmp(i,:)]=OneDimDMPwithV(p_orign(:,i)',1,1,p_orign(end,i)*0.9,(p_orign(end,i)-p_orign(end-1,i))/0.001+0.5);%p_end = p_orign(end,i); v = (p_orign(end,i)-p_orign(end-1,i))/0.001
% end
[p_DMP(:,1:3),gtmp(1:3,:)]=OneDimDMPwithVv2(p_orign,1,1,1.01*p_orign(:,end),0.2+(p_orign(:,end)-p_orign(:,(end-1)))/0.001);
%��̬DMP
R_orign=tr2rpy(T)';
% for i=1:3    
%     [R_DMP(:,i),gtmp(i+3,:)]=OneDimDMPwithV(R_orign(:,i)',1,1,R_orign(end,i),(R_orign(end,i)-R_orign(end-1,i))/0.001);
% end
[R_DMP(:,1:3),gtmp(4:6,:)]=OneDimDMPwithVv2(R_orign,1,1,R_orign(:,end),(R_orign(:,end)-R_orign(:,(end-1)))/0.001);
%% ��ν�DMP�õ��Ĺ켣���˶����ؽڿռ�
%1���ҵ�Ŀ���͵�ǰ���λ��
%��ʼ���λ��T(:,:,1)
%DMPѧϰ���λ��

if ToolboxWay==1
    T_DMP=zeros(size(T));
    for i=1:size(p_DMP,1)
        T_DMP(:,4,i)=[p_DMP(i,:)';1];
        if RotFlag==1
            T_DMP(1:3,1:3,i)=rpy2r(R_DMP(i,:));
        else
            T_DMP(1:3,1:3,i)=T(1:3,1:3,i);
        end
    end
    nPoint=5;
    q(1,:)=HomeArmR.ikine(T_DMP(:,:,1));
    q(nPoint-3,:)=HomeArmR.ikine(T_DMP(:,:,round((nPoint-4)*end/(nPoint-1))),'pinv');
    q(nPoint-2,:)=HomeArmR.ikine(T_DMP(:,:,round((nPoint-3)*end/(nPoint-1))),'pinv');
    q(nPoint-1,:)=HomeArmR.ikine(T_DMP(:,:,round((nPoint-2)*end/(nPoint-1))),'pinv');
    q(nPoint,:)=HomeArmR.ikine(T_DMP(:,:,end),'pinv');
else
    T_DMP=T;
    for i=1:size(p_DMP,1)
        T_DMP(:,4,i)=[p_DMP(i,:)';1];
    end
    nPoint=2;
    [q(:,:,1),isOK(1,1)]=home_IKNumSolution(transl(T_DMP(:,:,2)),t2r(T_DMP(:,:,2)), qr(1,:)');
    [q(:,:,nPoint),isOK(nPoint,1)]=home_IKNumSolution(transl(T_DMP(:,:,nPoint)),t2r(T_DMP(:,:,nPoint)), qr(nPoint-1,:)');
    q=permute(q,3,1,2);
end

%% ��q���в�ֵ
t=(0:1/(nPoint-1):1)';
q= [q,zeros(size(q,1),7)];
[ss,tt]=cubicSplineVnA(q,t,0.001);
plot(t,q,'o',tt,ss)
q=ss(:,1:7);

% q=zeros(7,1,size(ql,1));
% isOK=ones(size(ql,1),1);
% for i=1:size(ql,1)
%     [q(:,:,i),isOK(i,1)]=home_IKNumSolution(transl(T(:,:,i)),t2r(T(:,:,1)), ql(i,:)'); 
% %     [q(:,:,i),isOK(i,1)]=home_IKNumSolution_main(transl(T(:,:,i)),t2r(T(:,:,1)));
% end
%% ����
figure
workDims=[-reach reach -reach reach -0.2 reach]*0.8;
axis(workDims)
set(gca, 'Zdir', 'reverse','drawmode', 'fast'); view(145,12);
set(gcf,'Position',[80 80 700 600])%[a b c d] abΪfigure����Ļ�ϵ�ԭ�㣬�����½�Ϊ׼��c dΪfigure ͼ��ߴ�
plotopt=HomeArmR.plot({'noshadow'});
hold on
% ����ߴ�,�ֱ���xyz��
a=0.095;
b=0.09;
c=0.1;
[vertCell faceCell colorCell]=blockSurf([-a/2,-b/2,0.01],a,b,c,'m');
p0=patch('Vertices',vertCell,'Faces',faceCell,'FaceVertexCData',colorCell,'FaceColor','flat');
% color=['b','g','r'];
% for i=1:1700:5100
%     plot3(t(i:i+1700-1,1)',t(i:i+1700-1,2)',t(i:i+1700-1,3)',color(fix(i/1700)+1))
% end 
grid on
%ԭʼ�켣
plot3(p_orign(1,:)',p_orign(2,:)',p_orign(3,:)','r')
hold on
%���ɹ켣
plot3(p_DMP(:,1)',p_DMP(:,2)',p_DMP(:,3)','.g')
hold on
%��ʼλ��
plot3(gtmp(1,end),gtmp(2,end),gtmp(3,end),'k*');
hold on
%�ϰ���
plot3(obstacle(1),obstacle(2),obstacle(3),'b*');
hold on
for i=1:10:size(q,1)
    plot3(gtmp(1,i),gtmp(2,i),gtmp(3,i),'c*');
    hold on
    HomeArmR.plot(q(i,:),plotopt);
    hold on
    if SaveGif==1
        MakeGif('DMP.gif',i);
    end
end
hold on
plot3(gtmp(1,end),gtmp(2,end),gtmp(3,end),'c*');




