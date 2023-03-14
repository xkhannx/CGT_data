clear;
close all; clc;

figure(1); hold on; grid on

fileNames = dir(['11-Dec-2021' '/*.mat']);
colMat = get(gca,'colororder')';

th0 = [];
nozAng = [];
color = 1;
for ind = [3, 5, 7, 8, 9, 10, 11, 12, 14, 15]%% Plot data
    load([fileNames(ind).folder, '/test', num2str(ind)]);
    
    qB = EXP_theta.signals(1).values(:, 2);
    dqB = EXP_theta.signals(2).values(:, 2);
    qFuture = EXP_theta.signals(1).values(:, 1);
    dqFuture = EXP_theta.signals(2).values(:, 1);
    
    tB = linspace(0, length(qB)/1000, length(qB));
    noz = EXP_nozzleAng.signals.values(:, 2);
    nozRef = EXP_nozzleAng.signals.values(:, 1);

    plot(qB, lowpass(dqB, 1, 1000), 'Color', [colMat(:, color)', 0.5]);
    color = color + 1;
    if (color > length(colMat)) 
        color = 1;
    end

    plot(qB(10000), dqB(10000), 'or', 'LineWidth', 2)
    plot(qB(end), dqB(end), 'xk', 'LineWidth', 2)
end

xlim([-12, 20]);
ylim([-25, 60]);
xlabel('Block Angle (deg)');
ylabel('Block Vel (deg/s)');
% plot(qB(10000), control_modifiers.signals(3).values(10000), 'xr');
% plot(qFuture(10000), dqFuture(10000), 'or', 'LineWidth', 2)
% plot(qFuture(10000), control_modifiers.signals(3).values(10000), 'xr');
