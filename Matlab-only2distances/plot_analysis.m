t=0:0.1:100;
% 
% r = 2; % Radius
% fig = figure;
% hold on
% th = 0:pi/100:2*pi;
% xunit = xA(900,1) + r * cos(th);
% yunit = yA(900,1) + r * sin(th);
% plot(xunit, yunit);

figure;
plot(xA(1:700,1),yA(1:700,1),xA2(1:700,1),yA2(1:700,1),xA3(1:700,1),yA3(1:700,1));
hold on
plot(xA(700,1),yA(700,1),'-o','MarkerFaceColor',[1 .6 .6])
plot(xA2(700,1),yA2(700,1),'-o','MarkerFaceColor',[1 .6 .6])
plot(xA3(700,1),yA3(700,1),'-o','MarkerFaceColor',[1 .6 .6])
plot(xA(1,1),yA(1,1),'-o','MarkerFaceColor',[1 .6 .6])
plot(xA2(1,1),yA2(1,1),'-o','MarkerFaceColor',[1 .6 .6])
plot(xA3(1,1),yA3(1,1),'-o','MarkerFaceColor',[1 .6 .6])
hold off
xlabel('x (meters)')
ylabel('y (meters)')
title('Trajectory of UAVs')
axis([-110 160 -10 250])

figure;
subplot(3,1,1)
plot(t,ent1)
ylabel('S_1')
title('Tsallis Entropy from each UAV')
subplot(3,1,2)
plot(t,ent2)
ylabel('S_2')
subplot(3,1,3)
plot(t,ent1)
ylabel('S_3')
xlabel('Time (sec)')


%fdi_false_alarm_prob_airs=(length(find(fdi_alarm_airs==1)))/length(fdi_alarm_airs)


figure;
subplot(2,1,1);plot(t,fdi_alarm_airs,'b+');
ylabel('Alarm index')
title('Alarm and fault index of FDI')
subplot(2,1,2);plot(t,fdi_fault_index_airs,'b+');
ylabel('Fault index')
xlabel('Time (sec)')
axis([0 100 0 3])