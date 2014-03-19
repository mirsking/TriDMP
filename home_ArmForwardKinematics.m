function T=home_ArmForwardKinematics(Theta,q_index)
%Theta: shoulder pitch roll yaw
n=length(Theta);
%ÊÖ±Û²ÎÊý
L0=0.142;
L1=0.281;
L2=0.2125;
L3=0.200;
if n~=7
    disp('error dim');
    return
end
switch q_index
    case 0
        T=r2t(rotx(0));
    case 1
        T=r2t(rotz(Theta(1)));
    case 1.5
        T=r2t(rotz(Theta(1)))*[eye(3),[0 -L0 0]';0 0 0 1];
    case 2
        Tn{1}=r2t(rotz(Theta(1)))*[eye(3),[0 -L0 0]';0 0 0 1];
        T=Tn{1}*r2t(roty(Theta(2)));
    case 3
        Tn{1}=r2t(rotz(Theta(1)))*[eye(3),[0 -L0 0]';0 0 0 1];
        Tn{2}=Tn{1}*r2t(roty(Theta(2)));
        T=Tn{2}*r2t(rotx(Theta(3)));
    case 4
        Tn{1}=r2t(rotz(Theta(1)))*[eye(3),[0 -L0 0]';0 0 0 1];
        Tn{2}=Tn{1}*r2t(roty(Theta(2)));
        Tn{3}=Tn{2}*r2t(rotx(Theta(3)));
        T=Tn{3}*r2t(rotz(Theta(4)));
    case 5
        Tn{1}=r2t(rotz(Theta(1)))*[eye(3),[0 -L0 0]';0 0 0 1];
        Tn{2}=Tn{1}*r2t(roty(Theta(2)));
        Tn{3}=Tn{2}*r2t(rotx(Theta(3)));
        Tn{4}=Tn{3}*r2t(rotz(Theta(4)));
        T=Tn{4}*[rotx(Theta(5)) [0 0 -L1]';0 0 0 1];
    case 6
        Tn{1}=r2t(rotz(Theta(1)))*[eye(3),[0 -L0 0]';0 0 0 1];
        Tn{2}=Tn{1}*r2t(roty(Theta(2)));
        Tn{3}=Tn{2}*r2t(rotx(Theta(3)));
        Tn{4}=Tn{3}*r2t(rotz(Theta(4)));
        Tn{5}=Tn{4}*[rotx(Theta(5)) [0 0 -L1]';0 0 0 1];
        T=Tn{5}*[rotz(Theta(6)) [0 0 -L2]';0 0 0 1];
    case 7
        Tn{1}=r2t(rotz(Theta(1)))*[eye(3),[0 -L0 0]';0 0 0 1];
        Tn{2}=Tn{1}*r2t(roty(Theta(2)));
        Tn{3}=Tn{2}*r2t(rotx(Theta(3)));
        Tn{4}=Tn{3}*r2t(rotz(Theta(4)));
        Tn{5}=Tn{4}*[rotx(Theta(5)) [0 0 -L1]';0 0 0 1];
        Tn{6}=Tn{5}*[rotz(Theta(6)) [0 0 -L2]';0 0 0 1];
        T=Tn{6}*r2t(rotx(Theta(7)));
    case 8
        Tn{1}=r2t(rotz(Theta(1)))*[eye(3),[0 -L0 0]';0 0 0 1];
        Tn{2}=Tn{1}*r2t(roty(Theta(2)));
        Tn{3}=Tn{2}*r2t(rotx(Theta(3)));
        Tn{4}=Tn{3}*r2t(rotz(Theta(4)));
        Tn{5}=Tn{4}*[rotx(Theta(5)) [0 0 -L1]';0 0 0 1];
        Tn{6}=Tn{5}*[rotz(Theta(6)) [0 0 -L2]';0 0 0 1];
        Tn{7}=Tn{6}*r2t(rotx(Theta(7)));
        Tn{8}=Tn{7}*[eye(3) [0 0 -L3]';0 0 0 1];
        T=Tn{8};
    otherwise
        T=eye(3);
end



