%% MakeGif save the pictures in the figure window to a dynamic photo.
% example:
%    MakeGif(filename,i) 
%    filename is photo's name you want to save with a extension name of
%    gif.
%    i means the ith picture of the dynamic photo. That's to say when
%    you plot the ith picture, use MakeGif(filename,i) to save it, and
%    after you finish the plot, the photo will contain all your picture.
function MakeGif(filename,i)
    f = getframe(gcf);
    imind = frame2im(f);
    [imind,cm] = rgb2ind(imind,256);
    if i==1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.05);%感觉时间太短改这个，但是储存就很卡
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.1);%感觉时间太短改这个，但是储存就很卡
    end
end