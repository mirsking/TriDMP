clc ;clear
x = linspace(-2,2,200);%200���ֵԽ������Խϸ��
[X,Y,Z] = meshgrid(x,x,x);
I1 = (X.^2+9/4*Y.^2+Z.^2-1).^3-X.^2.*Z.^3-9/80*Y.^2.*Z.^3;
p = patch(isosurface(X,Y,Z,I1,0));
set(p, 'FaceColor', 'red', 'EdgeColor', 'none');
view(3);
axis equal ;
axis off;
light('Posi',[0 -2 3]); % ��(0,-2,3)�㴦����һ����Դ
lighting phong
set(gca,'nextplot','replacechildren');
% ��¼��Ӱ
XX = get(p,'XData');
YY = get(p,'YData');
ZZ = get(p,'ZData');
for j = 1:20
    scale = sin(pi*j/20);
    set(p,'XData',scale*XX,'YData',scale*YY,'ZData',scale*ZZ)
    drawnow
    MakeGif('heart-beats.gif',j)
end

