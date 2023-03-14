clear; clc; close all;

figure(1);
subplot 211; hold on; grid on;
subplot 212; hold on; grid on;

colMat = get(gca,'colororder');
colMat(1, :) = [0, 0, 0];
colMat(2, :) = [0.6, 0.6, 0.6];

starts =        [   1;  527;  486;  555;  672;  401;  576;  739;  455;  420];
trial_numbers = ["01"; "02"; "03"; "04"; "06"; "08"; "10"; "11"; "19"; "20"];
ends =          [  -1;  975;   -1;   -1;   -1;   -1;   -1;  845;  690;   -1];
offsets =       [   1;    1;    1; 0.78; 0.65;    0;    0; 0.88;    1;  0.6];

trials = cell(1, length(starts));

comments = [
    "Static 5.8 deg F";
    "Static 3.8 deg S";
    "Slow F";
    "Slow S";
    "Rock over S";
    "Rock over F";
    "Static 10 deg F"; % 10
    "Rock over S";
    "Zero push F";
    "Zero push S"];

inds = 1:length(starts);
for ind = inds

    filename = append('CGT_off/block_noThrust_121521_', trial_numbers(ind), '.c3d');

    h = btkReadAcquisition(convertStringsToChars(filename));
    markers = btkGetMarkers(h);

    aX = markers.base3(:, 1);
    aY = markers.base3(:, 3);
    aZ = markers.base3(:, 2);

    bX = markers.base4(:, 1);
    bY = markers.base4(:, 3);
    bZ = markers.base4(:, 2);

    dX = sqrt( (bX-aX).^2 + (bZ-aZ).^2);
    dY = bY - aY;

    fc = 5;
    q = atan2d(dY, dX) + offsets(ind);
    q = [ones(999, 1) * q(1); q; ones(1000, 1) * q(end)];

    q1 = q(1) + lowpass(q - q(1), 5, 200);

    dq = [0; diff(q1)]*200;
    dq1 = dq(1) + lowpass(dq - dq(1), 0.1, 200);
    dq2 = zeroPhaseKinematics(dq, 200, fc); dq2 = dq2(:, 1);

    if ends(ind) == -1
        pickInds = (1000 + starts(ind)):(length(q) - 1000);
    else
        pickInds = (1000 + starts(ind)):(starts(ind) + ends(ind) + 1000);
    end

    q = q(pickInds);
    q1 = q1(pickInds);
    dq = dq(pickInds);
    dq1 = dq1(pickInds);
    dq2 = dq2(pickInds);

    t = (0:length(q)-1)' * 0.005;

    subplot(2, 1, 2)

    if q1(end) < 10
        plot(t, q1, 'Color', [colMat(1, :) 0.6]);
    else
        plot(t, q1, '--', 'Color', [colMat(1, :) 0.6]);
    end
%     text(t(70), q1(70), num2str(trial_numbers(ind)))

    subplot(2, 1, 1)

    if q1(end) < 10
        plot(q1, dq2, 'Color', [colMat(1, :) 0.6]);
        plot(q1(1), dq2(1), 'x', 'Color', colMat(1, :), 'LineWidth', 2);
    else
        plot(q1, dq2, '--', 'Color', [colMat(1, :) 0.6]);
        plot(q1(1), dq2(1), 'x', 'Color', colMat(2, :), 'LineWidth', 2);
    end
%     text(q1(1), dq2(1), num2str(trial_numbers(ind)))

    trials{ind}.t = t;
    trials{ind}.q = q1;
    trials{ind}.dq = dq2;
    trials{ind}.filename = filename;
    trials{ind}.number = trial_numbers(ind);
end
%%
subplot(2, 1, 2)
xlabel('Time (s)');
ylabel('Block Angle (deg/s)');
xlim([0, 4]);
ylim([-5, 15]);

subplot(2, 1, 1)
title("Without CGT")
xlabel('Block Angle (deg)');
ylabel('Block Vel (deg/s)');
xlim([-6, 15]);
ylim([-25, 50]);

plot([0, 4], [19, 0], 'k', 'LineWidth', 2)
plot([0, 15.25], [47.236, 0], 'k--', 'LineWidth', 1.5);