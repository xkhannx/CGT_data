function stick_figure(stateData, com_ang, comLen)
colMat = get(gca,'colororder');
h = 1.55;
legLen = h * (0.53 - 0.039);
trunkLen = h * (1 - 0.53);
for ind = [1, length(comLen)]
    if ind == 1
        alpha = 1;
    else
        alpha = 0.3;
    end

    % STICK FIGURE
    ank = stateData.data(ind, 2)*180/pi;
    hip = stateData.data(ind, 4)*180/pi;

    % Foot
    plot([0, 0.152/3, -0.152*2/3, 0]*h, [0.039, 0, 0, 0.039]*h, 'LineWidth', 2, 'Color', [colMat(1, :) alpha])
    % Leg
    plot([0, legLen * sind(ank), legLen * sind(ank) + trunkLen * sind(ank + hip)], 0.039*h + ...
        [0, legLen * cosd(ank), legLen * cosd(ank) + trunkLen * cosd(ank + hip)], 'LineWidth', 2, 'Color', [colMat(2, :) alpha]);
    % Nozzle
    if ind == 1
        nozX = legLen * sind(ank) + (h * 0.75 - legLen) * sind(ank + hip);
        nozY = legLen * cosd(ank) + (h * 0.75 - legLen) * cosd(ank + hip);
        plot([nozX, nozX + 0.5 * cosd(com_ang(ind))], 0.039*h + [nozY, nozY - 0.5 * sind(com_ang(ind))], 'LineWidth', 3);
%         text (0.22, 1.4, ['Noz ang: ', num2str(90 + floor(ank + hip - com_ang(ind))), ' deg']);
    end

    % COM pendulum
    plot([0, comLen(ind) * sind(com_ang(ind))], [0, comLen(ind) * cosd(com_ang(ind))], ...
        '--', 'LineWidth', 2, 'Color', [colMat(4, :) alpha]);
    plot(comLen(ind) * sind(com_ang(ind)), comLen(ind) * cosd(com_ang(ind)), 'x', 'LineWidth', 2, 'Color', [colMat(4, :) alpha],'MarkerSize', 9);
    plot(comLen(ind) * sind(com_ang(ind)), comLen(ind) * cosd(com_ang(ind)), 'o', 'LineWidth', 2, 'Color', [colMat(4, :) alpha],'MarkerSize', 9);
    % scatter(comLen(ind) * sind(com_ang(ind)), comLen(ind) * cosd(com_ang(ind)), 300, 'MarkerEdgeColor', colMat(4, :), 'MarkerEdgeAlpha', alpha);
    % scatter(comLen(ind) * sind(com_ang(ind)), comLen(ind) * cosd(com_ang(ind)), 300, 'MarkerEdgeColor', colMat(4, :), 'MarkerEdgeAlpha', alpha);

end

axis equal