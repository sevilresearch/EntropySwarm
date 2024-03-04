t=0:0.1:500;
% 
% r = 2; % Radius
% fig = figure;
% hold on
% th = 0:pi/100:2*pi;
% xunit = xA(900,1) + r * cos(th);
% yunit = yA(900,1) + r * sin(th);
% plot(xunit, yunit);
EndP=length(xA);%7000;
figure;
plot(xA(1:EndP,1),yA(1:EndP,1),xA2(1:EndP,1),yA2(1:EndP,1),xA3(1:EndP,1),yA3(1:EndP,1),'linewidth',2);
hold on
plot(xA(EndP,1),yA(EndP,1),'-o','MarkerFaceColor',[1 .6 .6],'MarkerSize',10)
plot(xA2(EndP,1),yA2(EndP,1),'-o','MarkerFaceColor',[1 .6 .6],'MarkerSize',10)
plot(xA3(EndP,1),yA3(EndP,1),'-o','MarkerFaceColor',[1 .6 .6],'MarkerSize',10)
plot(xA(1,1),yA(1,1),'-o','MarkerFaceColor',[1 .6 .6],'MarkerSize',10)
plot(xA2(1,1),yA2(1,1),'-o','MarkerFaceColor',[1 .6 .6],'MarkerSize',10)
plot(xA3(1,1),yA3(1,1),'-o','MarkerFaceColor',[1 .6 .6],'MarkerSize',10)
hold off
xlabel('x (meters)','fontsize',14)
ylabel('y (meters)','fontsize',14)
title('Trajectory of UAVs','fontsize',16)
grid on
%axis([-110 160 -10 150])

figure;
subplot(3,1,1)
plot(t,ent1)
grid on
ylabel('S_1')
title('Tsallis Entropy from each UAV')
subplot(3,1,2)
plot(t,ent2)
grid on
ylabel('S_2')
subplot(3,1,3)
plot(t,ent3)
grid on
ylabel('S_3')
xlabel('Time (sec)')

figure;
subplot(3,1,1)
plot(t,fdi_airspeed(:,1),'linewidth',2)
grid on
ylabel('S_1','fontsize',14)
title('Tsallis Entropy from each UAV','fontsize',16)
subplot(3,1,2)
plot(t,fdi_airspeed(:,2),'linewidth',2)
grid on
ylabel('S_2','fontsize',14)
subplot(3,1,3)
plot(t,fdi_airspeed(:,3),'linewidth',2)
grid on
ylabel('S_3','fontsize',14)
xlabel('Time (sec)','fontsize',14)


fdi_false_alarm_prob_airs=(length(find(fdi_alarm_airs==1)))/length(fdi_alarm_airs)


figure;
subplot(2,1,1);plot(t,fdi_alarm_airs,'b+');
grid on
ylabel('Alarm index','fontsize',14)
title('Alarm and fault index','fontsize',16)
subplot(2,1,2);plot(t,fdi_fault_index_airs,'b+');
ylabel('Fault index','fontsize',14)
xlabel('Time (sec)','fontsize',14)
grid on
%axis([0 100 0 3])