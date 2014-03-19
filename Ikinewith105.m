
T=HomeArmR.fkine(ql);
q=zeros(7,1,size(ql,1));
isOK=ones(size(ql,1),1);
for i=1:1000:size(ql,1)
    [q(:,:,i),isOK(i,1)]=home_IKNumSolution(transl(T(:,:,i)),t2r(T(:,:,1)), ql(i,:)'); 
%     [q(:,:,i),isOK(i,1)]=home_IKNumSolution_main(transl(T(:,:,i)),t2r(T(:,:,1))); 
end