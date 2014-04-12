%% �޸ı��Ϻ������ʺ϶�ά����

function [y,gm]=OneDimDMPwithVv3(x,t_dem,t_run,g_m,v_m)%i_obstacle
%%
%     ������ǣ�1��ʾ�̹켣 ����dem_time x xd xdd
%               2) �滮�켣��ʱ�� traj_time
%               3) �˶�Ŀ��켣 g_m

    %% ��������ʽ
    % x  = (tpoly(0,1,1001))';
    % xd = diff(x)/dt;
    % xd = [xd xd(end)];
    % xdd = diff(xd)/dt;
    % xdd = [xdd xdd(end)];
    dt=0.001;
    x_dim = size(x,1);
    xd = diff(x,1,2);
    xd =[xd, xd(:,end)]/dt;
    xdd = diff(xd,1,2);
    xdd=[xdd,xdd(:,end)]/dt;
    dem_time=t_dem;
    t_dem=0:dt:t_dem;

    %% DMPģ�ͳ�ʼ��
    dmp.D = 15;
    dmp.K = dmp.D.^2/4;
    dmp.tau=t_dem(end)-t_dem(1);  %dmpʱ���������
    dmp.x0=x(:,1);
    dmp.g=x(:,end);
%     dmp.A=x(:,end)-x(:,1);
    dmp.alpha=4;
    % Ȩ�غ�����ʼ��
    n_w=length(t_dem)/10;   %Ȩ�غ������� ȡ100��������Խ�ྫ��Խ�ߡ����ֵΪѵ�����ݸ�����
    dmp.wc=exp(-dmp.alpha/dmp.tau*(0:1/(n_w-1):1)'); %100*1  Ȩ�غ������ģ�ע�����ڴ�tӳ�䵽s������ҲҪ��Ȩ�غ�������ȡ����
    dmp.wh=(diff(dmp.wc)*0.65).^2;  %99*1  Ȩ�غ����Ŀ����ǰ������ļ����ѡ���
    dmp.wh=1./[dmp.wh;dmp.wh(end)]; %100*1

    % �淶����
    % dmp.s = exp((-dmp.alpha/dmp.tau).*t_dem); %1*1001  %ʵ�⣬�������������ʹdmp.psi���ﵽ3��ô��
    dmp.s = zeros(1,size(x,2));
    dmp.s(1) = 1;
    for i = 2:length(x),
        dmp.s(i) = dmp.s(i-1)-dmp.alpha*dmp.s(i-1)*dt/dmp.tau;
    end
    %% w(i)�ľֲ���Ȩѧϰ
    % - weighting functions
    for i=1:x_dim
        dmp.psi(:,:,i) = exp( -dmp.wh*ones(1,length(x)) .* ( ( ones(length(dmp.wc),1)*dmp.s(1,:) - dmp.wc*ones(1,length(x)) ).^2 ) );   %100*1001
        dmp.vm(i,:)=dmp.tau*xd(i,end);
        dmp.gm(i,:)=x(i,end)-dmp.vm(i,:)*dem_time+dmp.vm(i,:)*t_dem;
        dmp.f(i,:) = (dmp.tau.^2*xdd(i,:)-dmp.K*(dmp.gm(i,:)-x(i,:))-dmp.D*(dmp.vm(i,:)-dmp.tau*xd(i,:))+dmp.K*(dmp.gm(i,:)-dmp.x0(i,1)).*dmp.s(1,:) )/dmp.K;   %1*1001

        % locally weighted linear regression
        wDen(:,:,i) = sum( dmp.psi(:,:,i) .* ( ones(length(dmp.wc),1)*(dmp.s(1,:).^2) ) ,2);
        wNum(:,:,i) = sum( dmp.psi(:,:,i) .* ( ones(length(dmp.wc),1)*(dmp.s(1,:).*dmp.f(i,:)) ) ,2);
        dmp.w(:,:,i) = wNum(:,:,i)./wDen(:,:,i);%+1.e-10
    end
    %% ѧϰ��켣����
    % dmp������ʼ��
    traj_time=t_run;
    t_run=0:dt:t_run;
    dmp.tau=t_run(end)-t_run(1);  %dmpʱ���������
    dmp.x0=x(:,1);
    dmp.g=g_m;
    dmp.vm=v_m;
%     dmp.A=dmp.g-x(:,1);
    dmp.s1=1*ones(x_dim,1);
    % dmp.s1 = exp((-dmp.alpha/dmp.tau).*t_run);
    dmp.x1=x(:,1);
    dmp.v1=dmp.tau*xd(:,1);
    for i=1:length(t_run)
        for j=1:x_dim
            dmp.gm(j)=dmp.g(j)-dmp.vm(j)*traj_time+dmp.vm(j)*t_run(i);
            dmp.tmp(j,i)=dmp.gm(j);
            dmp.psi1(:,j) = exp(-dmp.wh.*((dmp.s1(j)-dmp.wc).^2));
            dmp.f1(j) = sum(dmp.psi1(:,j).*dmp.w(:,:,j)*dmp.s1(j))/sum(dmp.psi1(:,j)+1.e-10);       
            if i==1
                dp=evalin('base','dp');
                dmp.v1_d(j)=(dmp.K*(dmp.gm(j)-dmp.x1(j))+dmp.D*(dmp.vm(j)-dmp.v1(j))-dmp.K*(dmp.gm(j)-dmp.x0(j))*dmp.s1(j)+dmp.K*dmp.f1(j)+obstacle(dmp.x1,dmp.v1,dp,j))/dmp.tau;                     
            else
                obs_bk(i,j)=obstacle(dmp.y(:,i-1),dmp.tau*dmp.yd(:,i-1),dp,j);
                dmp_bk(i,j)=dmp.K*(dmp.gm(j)-dmp.x1(j))+dmp.D*(dmp.vm(j)-dmp.v1(j))-dmp.K*(dmp.gm(j)-dmp.x0(j))*dmp.s1(j)+dmp.K*dmp.f1(j);
                dmp.v1_d(j)=(obs_bk(i,j)+dmp_bk(i,j))/dmp.tau;
%                 dmp.v1_d(j)=(dmp.K*(dmp.gm(j)-dmp.x1(j))+dmp.D*(dmp.vm(j)-dmp.v1(j))-dmp.K*(dmp.gm(j)-dmp.x0(j))*dmp.s1(j)+dmp.K*dmp.f1(j)+obstacle(dmp.y(:,i-1),dmp.tau*dmp.yd(:,i-1),j))/dmp.tau;
            end
            %����ѭ��x_dim�β��ܸ��±��ε�ȫ��������Ϣ��������ʱʹ����һ�εĵ���Ϊ�Ƴ������׼
            dmp.x1_d(j)=dmp.v1(j)/dmp.tau;
            dmp.x1_dd(j)=dmp.v1_d(j)/dmp.tau;
            dmp.s1d(j) = -dmp.alpha*dmp.s1(j)/dmp.tau;
            dmp.s1(j) = dmp.s1(j) + dt*dmp.s1d(j);
            dmp.x1(j)=dmp.x1(j)+dmp.x1_d(j)*dt;
            dmp.v1(j)=dmp.v1(j)+dmp.v1_d(j)*dt;

            %ÿ��ѭ���洢����
            dmp.y(j,i)=dmp.x1(j);
            dmp.yd(j,i)=dmp.x1_d(j);
            dmp.ydd(j,i)=dmp.x1_dd(j);
        end

    end
    y=dmp.y';
    gm=dmp.tmp;
    if nargout==0
%     % % y=[dmp.y',dmp.yd',dmp.ydd'];
        for i=1:x_dim
            figure;
            subplot(1,3,1);
            plot(t_dem,x(i,:),'.g');
            hold on
            plot(t_run,dmp.tmp(i,:),'k')
            hold on
            plot(t_run,dmp.y(i,:),'r');

            subplot(1,3,2);
            plot(t_dem,xd(i,:),'.g');
            hold on
            plot(t_run,dmp.yd(i,:),'r');

            subplot(1,3,3);
            plot(t_dem,xdd(i,:),'.g');
            hold on
            plot(t_run,dmp.ydd(i,:),'r');
        end
    end
    
    save('tmp.mat','obs_bk','dmp_bk')
end




