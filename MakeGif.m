function MakeGif(i)
    filename = 'xizh.gif';
    f = getframe(gcf);
    imind = frame2im(f);
    [imind,cm] = rgb2ind(imind,256);
    if i==1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.05);%�о�ʱ��̫�̸���������Ǵ���ͺܿ�

    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.1);%�о�ʱ��̫�̸���������Ǵ���ͺܿ�
    end
end