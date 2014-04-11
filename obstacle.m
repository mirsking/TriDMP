clear all
close all
clc
v=[1,1,1];
p_obstacle=[0.5,0.5,0.5]';
x=0:0.1:1;
[x,y,z]=meshgrid(x,x,x);
for i=1:length(x)
    for j=1:length(x)
        for k=1:length(x)
            p=p_obstacle-[x(1,i,1),y(j,1,1),z(1,1,k)]';
            cos_th=v*p./(norm(v)*norm(p));
            if cos_th>0
                out(i,j,k)=(cos_th).^2*norm(v)./norm(p);
            else
                out(i,j,k)=0;
            end
        end
    end
end

[FX,FY,FZ]=gradient(out);

i=find

