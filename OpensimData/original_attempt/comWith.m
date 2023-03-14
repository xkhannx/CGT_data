clear; close all;

total = 10;

figure(1); hold on; grid on;
colMat = get(gca,'colororder');

offset = -0.0378819;
for fileInd = 1:total
    filename = ['with', num2str(fileInd), '.sto'];

    posData = importdata(filename);

    t = posData.data(:, 1);
    com_pos = posData.data(:, 32:33);

    com_pos(:, 1) = com_pos(:, 1) - offset;

    com_ang = atan2d(com_pos(:, 1), com_pos(:, 2));
    com_angvel = com_ang;

    for ind = 1:length(t)-1
        com_angvel(ind) = (com_ang(ind+1) - com_ang(ind)) / (t(ind+1) - t(ind));
    end
    com_angvel(end) = 0;

    if (abs(com_ang(end)) <= 2)
        col = [colMat(1, :) 0.6];
    else
        col = [colMat(2, :) 0.6];
    end

    plot(-com_ang, -com_angvel, 'Color', col);
    plot(-com_ang(1), -com_angvel(1), 'x', 'Color', col, 'LineWidth', 2);
%     text(-com_ang(1), -com_angvel(1), num2str(fileInd))
end

% plot([4, 4], [-20, 40], 'k--')
xlim([-6, 15])
ylim([-30, 70])

xlabel('COM Angle (deg)');
ylabel('COM Vel (deg/s)');

initAsymBlock;
thetas = 0:0.01:18;
maxWForce = maxVel_ansatz(thetas, Rblock, tank, 0.8) * 180/pi;
plot(thetas, max(0, maxWForce), 'k--', 'LineWidth', 1);

plot([0, 4], [19, 0], 'k--', 'LineWidth', 1)
plot([0, 4], [25.5, 0], 'k', 'LineWidth', 1)
plot([0, 4], [62.5, 35], 'k', 'LineWidth', 2)