function [s,w,j]=cubicSplineSmooth(q,t,wInv,u)
[n,N]=size(q);
wInv=diag(wInv,0);
lamda=(1-u)/6/u;
T=zeros(1,n-1);
for i=1:n-1
    T(i)=t(i+1)-t(i);
end
La=[2*T 0]+[0 2*T];
A=diag(La,0)+diag(T,1)+diag(T,-1);
Lc=[-6./T 0]+[0 -6./T];
C=diag(Lc,0)+diag(6./T,1)+diag(6./T,-1);
w=inv(A+lamda.*C*wInv*C')*C*q;
s=q-lamda*wInv*C'*w;
for i=1:n-1
    j(i,:)=(w(i+1,:)-w(i,:))/T(i);
end