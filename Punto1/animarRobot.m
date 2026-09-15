function animarRobot(Qtray, q06, robot)
n = size(Qtray,1);
fig = figure('Color','w', 'Name','KUKA KR10 R1100-2');
ax = axes(fig);
hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');
view(ax,35,25);

plot3(ax, q06(:,1), q06(:,2), q06(:,3), '--', ...
    'Color',[0.65 0.65 0.65], 'LineWidth',1);

hRobot = plot3(ax, nan, nan, nan, '-o', ...
    'Color',[0.95 0.35 0.05], 'MarkerFaceColor',[0.15 0.15 0.15], ...
    'LineWidth',4, 'MarkerSize',6);
hTray = animatedline(ax, 'Color',[0 0.35 0.85], 'LineWidth',2);
hX = plot3(ax,nan,nan,nan,'r','LineWidth',2);
hY = plot3(ax,nan,nan,nan,'g','LineWidth',2);
hZ = plot3(ax,nan,nan,nan,'b','LineWidth',2);

xlabel(ax,'X [mm]');
ylabel(ax,'Y [mm]');
zlabel(ax,'Z [mm]');
title(ax,'Trayectoria del KUKA KR10 R1100-2');
xlim(ax,[-1250 1350]);
ylim(ax,[-1250 1250]);
zlim(ax,[-200 1600]);

for i = 1:n
    [T06, Tglobal] = cinematicaDirecta(Qtray(i,:).', robot);
    puntos = zeros(3,7);

    for j = 1:6
        puntos(:,j+1) = Tglobal(1:3,4,j);
    end

    set(hRobot, 'XData',puntos(1,:), 'YData',puntos(2,:), ...
        'ZData',puntos(3,:));
    addpoints(hTray, T06(1,4), T06(2,4), T06(3,4));

    p = T06(1:3,4);
    R = T06(1:3,1:3);
    escala = 100;
    set(hX,'XData',[p(1),p(1)+escala*R(1,1)], ...
        'YData',[p(2),p(2)+escala*R(2,1)], ...
        'ZData',[p(3),p(3)+escala*R(3,1)]);
    set(hY,'XData',[p(1),p(1)+escala*R(1,2)], ...
        'YData',[p(2),p(2)+escala*R(2,2)], ...
        'ZData',[p(3),p(3)+escala*R(3,2)]);
    set(hZ,'XData',[p(1),p(1)+escala*R(1,3)], ...
        'YData',[p(2),p(2)+escala*R(2,3)], ...
        'ZData',[p(3),p(3)+escala*R(3,3)]);

    drawnow;
    pause(0.04);
end
end