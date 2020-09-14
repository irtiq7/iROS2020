% ************** TRAJECTORY SIMULATION**********************
% **********************************************************
% Change x,y for robot to follow a predefined trajectory


figure(666);
close all;
rectangle('Position',[1 0 4 15], 'LineStyle','--')
axis([-4 7 -4 16])
xlabel('X-Axis Distance [m]')
ylabel('Y-Axis Distance [m]')
box on
grid on
hold on

x = [2 2 2 2 2 2 2 2 3 4 4 4 4 4 4 4 4 4 3 2];
y = [2 2 4 6 8 10 12 12 12 12 12 10 8 6 4 2 2 2 2 2];
angleRobot = [90 90 90 90 90 90 90 0 0 0 270 270 270 270 270 270 270 180 180 180];

for ii = 1:length(x)
    angleWall = angleRobot(ii) + 90;
    plot(x(ii),y(ii), 'bo')
    hold on
    xEst(ii) = x(ii)+((1)*cosd(angleWall));
    yEst(ii) = y(ii)+((2.5)*sind(angleWall));
    plot(xEst(ii), yEst(ii), 'ro')
end
