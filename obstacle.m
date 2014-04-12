function F=obstacle(p,v,dp,dim)
%v��xΪ������
p_obstacle=evalin('base','obs');
dt = evalin('base','dt');
% xx=p(1)*linspace(0.9,1.1,10);
% yy=p(2)*linspace(0.9,1.1,10);
xx=linspace(0,1,10);
yy=xx;
[x,y]=meshgrid(xx,yy);
for i=1:length(x)
    for j=1:length(y)
        p=p_obstacle-[x(1,i),y(j,1)]';
        cos_th=v'*p./(norm(v)*norm(p));
        if cos_th>0
            out(i,j)=(cos_th).^2*norm(v)./norm(p);
        else
            out(i,j)=0;
        end       
    end
end
% figure
% surf(x,y,out)
% shading interp
% view(3)
[FX,FY]=gradient(out);
% FX=FX./dt;
% FY=FY./dt;
[tmp,i]=min(abs(xx-p(1)));
[tmp,j]=min(abs(yy-p(2)));
% F=[FX(i,j),FY(i,j)];
switch dim
    case 1
        F=FX(i,j)/(xx(2)-xx(1));
    case 2
        F=FY(i,j)/(yy(2)-yy(1));
end
end


