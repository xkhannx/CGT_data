clear; close all;
colMat = get(gca,'colororder');

total = 4;

for trial = 1:total
    folder = ['test', num2str(trial), '/'];
    posData = importdata([folder, 'test_pos.sto']);
    velData = importdata([folder, 'test_vel.sto']);
    stateData = importdata([folder, 'test_states.sto']);

    t = posData.data(:, 1);
    com_pos = posData.data(:, 32:33); com_pos(:, 1) = com_pos(:, 1) + 0.0378819;
    com_vel = velData.data(:, 32:33);

    ankAng = stateData.data(:, 2);
    ankVel = stateData.data(:, 3);

    hipAng = -stateData.data(:, 4);
    hipVel = -stateData.data(:, 5);

    th1 = ankAng - hipAng;
    w1 = ankVel - hipVel;

    xDot = -com_vel(:, 1);
    yDot = com_vel(:, 2);

    l1 = (0.56 - 0.53) * 1.54;
    l2 = (0.53 - 0.039) * 1.54;

    th2 = (yDot + l1 * th1 .* w1) ./ (l1 * w1 - xDot);

    comAng = (l1 * th1 + l2 * th2) / (l1 + l2);
    comVel = xDot / (l1 + l2);

    comAng_real = atan2(-com_pos(:, 1), com_pos(:, 2));
    comVel_real = zeros(size(comAng_real));

    for ind = 1:length(t)-1
        comVel_real(ind) = (comAng_real(ind+1) - comAng_real(ind)) / (t(ind+1) - t(ind));
    end

    comVel_real(end) = 0;

    figure(trial)
    subplot 231
    plot(t, [comAng, comAng_real] * 180/pi);

    grid on
    xlim([0, 0.6])
    xlabel('Time (s)')
    ylabel('COM Angle (deg)');
    title('COM angle (red:real, blue:est)')

    subplot 234
    plot(t, [comVel, comVel_real] * 180/pi);

    grid on
    xlim([0, 0.6])
    xlabel('Time (s)')
    ylabel('COM Vel (deg/s)');
    title('COM ang vel (red:real, blue:est)')

    subplot 232
    plot(t, [th2, ankAng] * 180/pi);

    grid on
    xlim([0, 0.6])
    xlabel('Time (s)')
    ylabel('Ankle Ang (deg)');
    title('Ankle Ang / th2 (red:real, blue:est)')

    subplot 235
    plot(t, [th1, comAng_real] * 180/pi);

    grid on
    xlim([0, 0.6])
    xlabel('Time (s)')
    ylabel('Ang (deg)');
    title('Trunk Ang (blue) vs real COM ang (red)')

    subplot 233
    plot(comAng(1:100) * 180/pi, comVel(1:100) * 180/pi);
    hold on
    plot(comAng_real * 180/pi, comVel_real * 180/pi);
    plot([0, 15], [48, 0])

    grid on
    xlim([0, 20])
    ylim([0, 50])
    xlabel('COM Angle (deg)')
    ylabel('COM Vel (deg/s)');
    title('COM phase plot (red:real, blue:est)')

    subplot 236
    plot(t, [ankAng, hipAng, th1] * 180/pi);

    grid on
    xlim([0, 0.6])
    xlabel('Time (s)')
    ylabel('Ang (deg)');
    title('Ankle (blue), Hip (red), Trunk (yellow) angle')
end