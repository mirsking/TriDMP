%%函数执行前要先设置 obstacle

function pot=psi_obstacle(x,v,dim)
	obstacle = evalin('base','obstacle');
    lambda=1;
    px=norm(x-obstacle);
	v_norm=norm(v);
    cosa=v'*x;
    pot=lambda*(-cosa)^2*v_norm/px;
end