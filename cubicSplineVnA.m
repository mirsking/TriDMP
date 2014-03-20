function [ss,tt]=cubicSplineVnA(q,t,Ts)
[n,N]=size(q);
if n > 2
    t=[t(1);(t(1)+t(2))/2;t(2:n-1);(t(n-1)+t(n))/2;t(n)];
else if n == 2
        t=[t(1);t(1)+(t(2)-t(1))/3;t(1)+(t(2)-t(1))*2/3;t(2)];
    end
end
T=zeros(1,n+1);
for i=1:n+1
    T(i)=t(i+1)-t(i);
end
La=[2*T(2:n) 4*T(n+1)]+[4*T(1) 2*T(2:n)];
A=diag(La,0)+diag([T(2:end-2) 0],1)+diag([0 T(3:end-1)],-1);
Lc=[-6./T(2:end-1) 0]+[0 -6./T(2:end-1)];
C=diag(Lc,0)+diag(6./T(2:end-1),1)+diag(6./T(2:end-1),-1);
w=inv(A)*C*q;
w=[zeros(1,N);w;zeros(1,N)];
q=[q(1,:);zeros(1,N);q(2:n-1,:);zeros(1,N);q(n,:)];
q(2,:)=q(1,:)+T(1)^2/6*w(2,:);
q(end-1,:)=q(end,:)+T(end)^2/6*w(end-1,:);
poly=zeros(4*(n+1),N);
for i=1:n+1
    poly(4*i-3:4*i,:)=[(w(i+1,:)-w(i,:))/6/T(i);w(i,:)/2;(q(i+1,:)-q(i,:))/T(i)-T(i)/6*(w(i+1,:)+2*w(i,:));q(i,:)];
end
tt=t(1);
ss=q(1,:);
for i=1:length(T)
    t_seq=(linspace(t(i)+Ts,t(i+1),round(T(i)/Ts)))';
    tt=[tt;t_seq];
    ss_temp=[];
    for j=1:N
        ss_temp=[ss_temp polyval(poly(4*i-3:4*i,j),t_seq-t(i))];
    end
    ss=[ss;ss_temp];
end