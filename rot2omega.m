function  w=rot2omega(R)
% calculation rot to omega
if norm(R-eye(3)) < 0.00001 
    w=[0 0 0]';
    return;
end

temp_theta=acos((R(1,1)+R(2,2)+R(3,3)-1)/2);
w=temp_theta/(2*sin(temp_theta))*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2);];
