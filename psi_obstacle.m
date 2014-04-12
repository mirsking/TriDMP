%%函数执行前要先设置 obstacle

function pot=psi_obstacle(x,v,dim)
	obstacle = evalin('base','obstacle');
    i_obstacle = evalin('base','i_obstacle');
    if i_obstacle==1
       pot=0;
       return;
    end
    lambda=1;
    
    a=linspace(min(x),max(x),5);
    [xx,yy,zz]=meshgrid(a,a,a);
    
    out = zeros(length(xx),length(yy),length(zz));
    for i=1:length(xx)
        for j=1:length(yy)
            for k=1:length(zz)
                p=obstacle-[xx(1,i,1),yy(j,1,1),zz(1,1,k)]';
                cos_th=v'*p./(norm(v)*norm(p));
                if cos_th>0
                    out(i,j,k)=lambda*(cos_th).^2*norm(v)./norm(p);
                else
                    out(i,j,k)=0;
                end
            end
        end
    end
    
   [FX,FY,FZ]=gradient(out);
   
   [tmp,i]=min(abs(a-x(1)));
   [tmp,j]=min(abs(a-x(2)));
   [tmp,k]=min(abs(a-x(3)));
   switch dim
       case 1
           pot=-FX(i,j,k);
       case 2
           pot=-FY(i,j,k);
       case 3
           pot=-FZ(i,j,k);
   end
   
end