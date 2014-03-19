function J = home_Jacobian(q)
ARM_DOF = 7;
if length(q)~=ARM_DOF
    disp('error dim');
    return
end
a_mat=[
    0 0 1 0 1 0 1
    0 1 0 0 0 0 0 
    1 0 0 1 0 1 0 ];
p_target = transl(home_ArmForwardKinematics(q,7));
for i=1:ARM_DOF
    aw = t2r(home_ArmForwardKinematics(q,i))*a_mat(:,i);
    pw = transl(home_ArmForwardKinematics(q,i));
    J(:,i) = [cross(aw,p_target-pw);aw];
end
return