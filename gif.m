%% 可以制作动态gif图

clc ;clear
x = linspace(-2,2,100);
[X,Y,Z] = meshgrid(x,x,x);
I1 = (X.^2+9/4*Y.^2+Z.^2-1).^3-X.^2.*Z.^3-9/80*Y.^2.*Z.^3;
p = patch(isosurface(X,Y,Z,I1,0));
set(p, 'FaceColor', 'red', 'EdgeColor', 'none');
view(3);
axis equal ;
axis off;
light('Posi',[0 -2 3]); % 在(0,-2,3)点处建立一个光源
lighting phong
set(gca,'nextplot','replacechildren');
% 记录电影
XX = get(p,'XData');
YY = get(p,'YData');
ZZ = get(p,'ZData');
% figure(1)
filename = 'hongxing1.gif';%保存的动画名
for j = 1:20
bili = sin(pi*j/20);
set(p,'XData',bili*XX,'YData',bili*YY,'ZData',bili*ZZ) 
F(j) = getframe;
%制作gif主要代码
drawnow
frame = getframe(gcf);
im = frame2im(frame);
[imind,cm] = rgb2ind(im,256);
if j == 1;
imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.1);
else
imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.1);
end
%
end
% 放映10次
figure
movie(F,10)