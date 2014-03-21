%%函数执行前要先设置 obstacle

function pot=psi_obstacle(x,v,dim)
	obstacle = evalin('base','obstacle');
    lambda=1;
    px=norm(x-obstacle);
	v_norm=norm(v);
    cosa=v'*x;
    if cosa>0
        pot=0;
    else
        pot=-lambda*(cosa)^2*v_norm/px;
    end
end