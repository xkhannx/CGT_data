% clear; close all;
offset = 0.00113139;

totals = [10, 10, 10, 5];

for fallMode = 1:3

    figure(1);
    subplot(2, 3, fallMode); 
    plot([4, 4], [-20, 70], 'k')
    hold on; grid on;
    switch(fallMode)
        case 1
            title('Straight body')
        case 2
            title('Vertical trunk')
        case 3
            title('Const hip ang = 40 deg')
    end
    
    xlim([0, 10])
    ylim([-15, 50])

    xlabel('COM Angle (deg)');
    ylabel('COM Vel (deg/s)');

    plot([0, 4], [19, 0], 'k', 'LineWidth', 2)
    plot([0, 15.25], [47.236, 0], 'k--', 'LineWidth', 1.5);

    for fileInd = 1:totals(fallMode)
        fileFolder = ['Mode', num2str(fallMode), '_new/', num2str(fileInd), '/'];

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

        if (abs(com_ang(end)) <= 2)
            col = 'k';
        else
            col = 'k--';
        end

        if fileInd >= 7
            if fileInd == 9
                plot(com_ang(1), com_angvel(1), 'sk', 'LineWidth', 1);
            end
        else
            plot(com_ang(1), com_angvel(1), 'xk', 'LineWidth', 2);
        end

        plot(com_ang, com_angvel, col);

        if (fileInd == 9)
            subplot(2, 3, fallMode + 3); hold on; grid on;

            stick_figure(stateData, com_ang, comLen);

            xlabel('x (m)');
            ylabel('y (m)');
            xlim([-0.4, 0.8])
            ylim([0, 1.6])
            subplot(2, 3, fallMode);
        end
    end
end

%%
sizeInches = [7 6];
res = 300;
set(gcf,'paperunits','inches','paperposition',[0 0 sizeInches]);
print('resized.tiff','-dtiff',['-r' num2str(res)]);