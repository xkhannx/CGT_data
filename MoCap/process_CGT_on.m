clear; clc; close all;

figure(1);
subplot 211; hold on; grid on;
subplot 212; hold on; grid on;

colMat = get(gca,'colororder');
colMat(1, :) = [0, 0, 0];

starts =        [ 232;  255;  176;   64;  956;  173;  377;  418;  467;  770;  363];
offsets =       [0.95;  0.9;    1;  0.7;  1.1; 1.07; 1.04;  1.3; 0.95; 0.92;    0];
fires  =        [ 106;   28;    1;   13;   34;   74;   26;   50;   21;  130;   12];
trial_numbers = ["08"; "10"; "12"; "13"; "17"; "18"; "19"; "20"; "22"; "23"; "21"];
ends =          [  -1;   -1;   -1;   -1;   -1;   -1;   -1; 1003;   -1;   -1;   -1];

trials = cell(1, length(starts));

comments = [
    "Slow";
    "Medium";
    "Fast";
    "Medium";
    "Slow, th0 = 9 deg";
    "Medium";
    "Fast";
    "Rock over, medium";
    "Rock over, fast";
    "Slow";
    "Rock over, too fast"];

%%
inds = 1:length(trial_numbers);

for ind = inds
    filename = append('CGT_on/block_121121_', trial_numbers(ind), '.c3d');

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
    plot(t-t(fires(ind)), q1, 'Color', [colMat(1, :) 0.6]);
    plot(0, q1(fires(ind)), 'ok', 'LineWidth', 2);

    subplot(2, 1, 1)
    plot(q1, dq2, 'Color', [colMat(1, :) 0.6]);
    plot(q1(1), dq2(1), 'xk', 'LineWidth', 2);
    plot(q1(fires(ind)), dq2(fires(ind)), 'ok', 'LineWidth', 2);
%     text(q1(1), dq2(1), trial_numbers(ind))

    trials{ind}.t = t;
    trials{ind}.q = q1;
    trials{ind}.dq = dq2;
    trials{ind}.filename = filename;
    trials{ind}.fireInd = fires(ind);
    trials{ind}.number = trial_numbers(ind);
end
%%
subplot(2, 1, 2)
xlabel('Time (s)');
ylabel('Block Angle (deg)');
xlim([-0.5, 4]);
ylim([-5, 15]);

subplot(2, 1, 1)
title("With CGT")
xlabel('Block Angle (deg)');
ylabel('Block Vel (deg/s)');

xlim([-6, 20]);
ylim([-25, 80]);

plot([0, 15.25], [47.236, 0], 'k--', 'LineWidth', 1.5);
plot([0, 4], [19, 0], 'k', 'LineWidth', 2)