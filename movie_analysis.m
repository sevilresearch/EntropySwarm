for iii=1:length(t);
% 
% r = 2; % Radius
% fig = figure;
% hold on
% th = 0:pi/100:2*pi;
% xunit = xA(900,1) + r * cos(th);
% yunit = yA(900,1) + r * sin(th);
% plot(xunit, yunit);
%EndP=length(xA);%7000;
figure;
plot(xA(1:iii,1),yA(1:iii,1),xA2(1:iii,1),yA2(1:iii,1),xA3(1:iii,1),yA3(1:iii,1));
hold on
plot(xA(iii,1),yA(iii,1),'-o','MarkerFaceColor',[1 .6 .6])
plot(xA2(iii,1),yA2(iii,1),'-o','MarkerFaceColor',[1 .6 .6])
plot(xA3(iii,1),yA3(iii,1),'-o','MarkerFaceColor',[1 .6 .6])
plot(xA(1,1),yA(1,1),'-o','MarkerFaceColor',[1 .6 .6])
plot(xA2(1,1),yA2(1,1),'-o','MarkerFaceColor',[1 .6 .6])
plot(xA3(1,1),yA3(1,1),'-o','MarkerFaceColor',[1 .6 .6])
hold off
xlabel('x (meters)')
ylabel('y (meters)')
title('Trajectory of UAVs')
axis([-110 160 -10 300])

M(iii) = getframe;
close all
end

movie(M,1,100)