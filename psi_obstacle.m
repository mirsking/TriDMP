function pot=psi_obstacle(x,v,i_obstacle)
    lambda=1;
    p=abs(x-x_obstacle);
    cosa=1;
    pot=lambda*(-cosa)^2*v/p;
end