function [q_target isOK]= home_IKNumSolution(p_target,R_target,q_0)
lamda = 0.6;
ARM_DOF = 7;

%%
iter=30;
q_crr=q_0;
Sp = 0.2;
Sw = 0.2;
while iter>0
    iter = iter-1;
    p_crr = transl(home_ArmForwardKinematics(q_crr,ARM_DOF));
    R_crr = t2r(home_ArmForwardKinematics(q_crr,ARM_DOF));
    delta_p =  p_target-p_crr;
%     if norm(delta_p) > Sp
%         delta_p = delta_p/norm(delta_p)*Sp;
%     end
    delta_R = R_target*(R_crr');
    delta_w = rot2omega(delta_R);
%     if norm(delta_w) > Sw
%         delta_w = delta_w/norm(delta_w)*Sw;
%     end
    delta_Cart = [delta_p;delta_w];
    err = norm(delta_Cart);
    if err<0.000001
        for i=1:ARM_DOF
            while abs(q_crr(i))>pi
                if q_crr(i)>0
                    q_crr(i) = q_crr(i) - 2*pi;
                else
                    q_crr(i) = q_crr(i) + 2*pi;
                end
            end
        end
        q_target = q_crr;
        isOK = 1;
%         fidout = fopen('home_arm_angles.txt','w');
%         zero7 = zeros(1,21);
%         fprintf(fidout,'%f\t',zero7);
%         for i=1:7
%             fprintf(fidout,'%f\t0\t0\t',q_target(i));
%         end
%         fclose(fidout);
        return
    end
    
    %delta_q = lamda*(pinv(home_Jacobian(q_crr))*delta_Cart);
    delta_q = lamda*(home_Jacobian(q_crr)\delta_Cart);
    q_crr = q_crr + delta_q;
end
q_target = zeros(ARM_DOF,1);
isOK = 0;
%disp('fail : iter over')




