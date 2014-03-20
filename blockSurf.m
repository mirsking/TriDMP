%blockSurf([x,y,z],a,b,c)
%[x,y,z] block ����ϵԭ��
%a, b, c �ֱ�Ϊ��x,y,z���ᣬblock�����
%�����ĸ��� �����ĸ��㣬��ʱ��˳��
%s ��ɫ
%��ɫ r ��ɫ g ��ɫ b ��ɫ k ��ɫ w �ۺ� m ��ɫ y ���� c
function [vert,face,color]=blockSurf(P,a,b,c,s)
x=P(1);
y=P(2);
z=P(3);
vert=[x y z
    x+a y z
    x+a y+b z
    x y+b z
    x y z+c
    x+a y z+c
    x+a y+b z+c
    x y+b z+c];
 face=[1 2 6 5;  
     2 3 7 6;
     3 4 8 7;
     4 1 5 8;
     1 2 3 4;
     5 6 7 8];
 switch s
     case 'r'
         color=repmat([1 0 0],6,1);
     case 'g'
         color=repmat([0 1 0],6,1);
     case 'b'
         color=repmat([0 0 1],6,1);
     case 'k'
         color=repmat([0 0 0],6,1);
     case 'w'
         color=repmat([1 1 1],6,1);
     case 'm'
         color=repmat([1 0 1],6,1);
     case 'y'
         color=repmat([1 1 0],6,1);
     case 'c'
         color=repmat([0 1 1],6,1);
     otherwise
         printf('color error!');         
 end


