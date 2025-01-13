function [toolframe] = moveBetweenPosRad(jtstart, jtend, TsFcn, vid, fig, vidfile)
% takes initial and end joint positions specified in radians
% and plots arm moving from first position to the next, saving to specified vidfile


%% in betweening

steps = 30;

jt = zeros(5, steps);
for i = 1:5
    jt(i,:) = linspace(jtstart(i), jtend(i), steps);
end

%% go through
for i = 1:steps
    clf(fig)
    toolframe = FK(jt(:,i), TsFcn, 1, fig);
    %view(vw)
    str = "FK: \theta_1 = "+ jt(1,i) + ", \theta_2 = "+jt(2,i)+", \theta_3 = "+ jt(3,i)+...
        ", \theta_4 = "+jt(4,i)+", \theta_5 = "+jt(5,i);
    title(str, 'Interpreter', 'tex')
    if vid
        writeVideo(vidfile,getframe(gcf));
    end
end


end
