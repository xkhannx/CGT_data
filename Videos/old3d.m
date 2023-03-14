clear; %close all

data3d = importdata("Frames\old\old2_3d.txt");

a1 = -data3d.data(:, 1);
a2 = -data3d.data(:, 2);
a3 = -data3d.data(:, 3);

m1 = 2 * 0.0465;
m2 = 2 * 0.1;
m3 = 0.678;

L1 = 0.246;
L2 = 0.245;
L3 = 0.288;

x1 = 0.567 * L1 * sind(a1);
x2 = L1 * sind(a1) + 0.567 * L2 * sind(a1 + a2);
x3 = L1 * sind(a1) + L2 * sind(a1 + a2) + 0.626 * L3 * sind(a1 + a2 + a3);

y1 = 0.567 * L1 * cosd(a1);
y2 = L1 * cosd(a1) + 0.567 * L2 * cosd(a1 + a2);
y3 = L1 * cosd(a1) + L2 * cosd(a1 + a2) + 0.626 * L3 * cosd(a1 + a2 + a3);

xCom = (m1 * x1 + m2 * x2 + m3 * x3) / (m1 + m2 + m3);
yCom = 0.039 + (m1 * y1 + m2 * y2 + m3 * y3) / (m1 + m2 + m3);
rCom = sqrt(xCom .^ 2 + yCom .^ 2);

comAng = atan2d(xCom, yCom);
% comVel = diff(comAng) / (1/30);
% comAng = comAng(2:end);

ang_3Hz = lowpass(atan2d(xCom, yCom), 3, 30);
vel_3Hz = diff(ang_3Hz) / (1/30);
ang_3Hz = ang_3Hz(2:end);

t = ((1:length(vel_3Hz)) - 1) / 30;

comVel = diff(comAng) / (1/30);
comAng = comAng(2:end);
%%
% figure(2)
% plot(comAng, comVel, 'LineWidth', 2)
% hold on
plot(ang_3Hz, vel_3Hz, 'k:', 'LineWidth', 2)
% plot(t, ang_3Hz, 'k:', 'LineWidth', 2)
% plot(ang_4Hz, vel_4Hz)
% plot([0, 15], [48, 0])
% plot([0, 4], [25.5, 0])
% ylabel('Vel (deg/s)')
% xlabel('Angle (deg)')
% legend('unfilt', '1Hz', '2Hz', '3Hz', '4Hz')

% figure(2)
% plot(t, ang)
% plot(t, vel_1Hz)
% hold on
% plot(t, a1)
% plot(t, -a3)
% plot(t, -(a1+a2+a3))
% legend("COM angle", "Ankle ang", "Hip angle", "Trunk angle");
% xlabel('Time (s)')
% ylabel('Angle (deg)')

% figure(2)
% plot(t, comAng, 'LineWidth', 2); hold on
% plot(t, ang_3Hz);
% % plot(t, [ang_1Hz, ang_2Hz, ang_3Hz, ang_4Hz])
% xlabel('Time (s)')
% ylabel('Angle (deg)')
% 
% figure(3)
% plot(t, comVel, 'LineWidth', 2); hold on
% plot(t, vel_3Hz);
% % plot(t, [vel_1Hz, vel_2Hz, vel_3Hz, vel_4Hz])
% ylabel('Vel (deg/s)')
% xlabel('Time (s)')