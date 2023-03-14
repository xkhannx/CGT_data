clear; colMat = get(gca,'colororder'); close all;
offset = 0.00113139;

col1 = [colMat(2, :) 0.6];
col2 = [colMat(4, :) 0.6];
col3 = [colMat(1, :) 0.6];

totals = [3 6 11 12];

for fallMode = 1:3

    subplot(2, 3, fallMode + 3);
    hold on; grid on;
    xlim([0, 1])
    ylim([0, 20])
    ylabel('COM Angle (deg)');
    xlabel('Time (s)');

    subplot(2, 3, fallMode);
    hold on; grid on;
    plot([0, 0], [-50, 50], 'Color', [0 0 0 0.3], 'LineWidth', 1)
    plot([-50, 50], [0, 0], 'Color', [0 0 0 0.3], 'LineWidth', 1)

    xlim([-3.5, 10])
    ylim([-25, 50])
    xlabel('COM Angle (deg)');
    ylabel('COM Vel (deg/s)');
    switch(fallMode)
        case 1
            title('Straight body')
        case 2
            title('Vertical trunk')
        case 3
            title('Const hip ang = 40 deg')
    end

    plot([0, 4], [19, 0], 'k--', 'LineWidth', 1)
    plot([0, 15.25], [47.236, 0], '--k', 'LineWidth', 1.5);

%     plotForwardSB;

    for fileInd = 1:length(totals)
        fileFolder = ['Mode', num2str(fallMode), '_new/', num2str(totals(fileInd)), '/'];

        posData = importdata([fileFolder, 'AnkleBody_BodyKinematics_pos_global.sto']);
        velData = importdata([fileFolder, 'AnkleBody_BodyKinematics_vel_global.sto']);
        stateData = importdata([fileFolder, 'ankleBody_states.sto']);

        t = posData.data(:, 1);
        com_pos = posData.data(:, 32:33);

        com_pos(:, 1) = com_pos(:, 1) - offset;
        com_vel = sqrt(velData.data(:, 32).^2 + velData.data(:, 33).^2) .* sign(velData.data(:, 32));

        com_ang = -atan2d(com_pos(:, 1), com_pos(:, 2));
        comLen = sqrt((com_pos(:, 1) - offset).^2 + com_pos(:, 2).^2);

        com_angvel = -com_vel ./ comLen *180/pi;

        if (fileInd <= 2)
            % Free fall region
            startInds = find(com_ang > 4);
            endInds = find(com_ang > 20);

            fire = find(com_angvel > 47.236 - 47.236 * com_ang / 15.25);

            subplot(2, 3, fallMode);
            plot(com_ang(startInds(1)), com_angvel(startInds(1)), 'x', 'Color', col1, 'LineWidth', 2);
            plot(com_ang(startInds(1):endInds(1)), com_angvel(startInds(1):endInds(1)), 'Color', col1, 'LineWidth', fileInd);
            plot(com_ang(fire(1)), com_angvel(fire(1)), 'o', 'Color', col1);

            subplot(2, 3, fallMode + 3);
            plot(t(startInds(1):endInds(1)), com_ang(startInds(1):endInds(1)), 'Color', col1, 'LineWidth', fileInd);
            plot(t(fire(1)), com_ang(fire(1)), 'o', 'Color', col1);

            % IMU estimation
            [comAngEst, comVelEst] = IMU_COM(velData.data(startInds(1):endInds(1), 32:33), stateData.data(startInds(1):endInds(1), :));

            fire = find(comVelEst > 47.236 - 47.236 * comAngEst / 15.25);

            subplot(2, 3, fallMode);
            plot(comAngEst, comVelEst, '--', 'Color', col2, 'LineWidth', fileInd);
            plot(comAngEst(1), comVelEst(1), 'x', 'Color', col2, 'LineWidth', 1);
            plot(comAngEst(fire(1)), comVelEst(fire(1)), 'o', 'Color', col2);

            subplot(2, 3, fallMode + 3);
            plot(t(startInds(1):endInds(1)), comAngEst, '--', 'Color', col2, 'LineWidth', fileInd);
            plot(t(fire(1) + startInds(1) - 1), comAngEst(fire(1)), 'o', 'Color', col2);
            plot(t(fire(1) + startInds(1) - 1), com_ang(fire(1) + startInds(1) - 1), '*', 'Color', col2)

            % CGT fire informed by IMU estimation
            subplot(2, 3, fallMode);
            plot(com_ang(fire(1) + startInds(1) - 1), com_angvel(fire(1) + startInds(1) - 1), '*', 'Color', col2);
        else
            % Plot CGT return
            subplot(2, 3, fallMode);
            plot(com_ang, com_angvel, 'Color', col3);
        end
    end
end
