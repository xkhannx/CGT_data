clear;
close all; clc;

figure(1); hold on; grid on

fileNames = dir(['11-Dec-2021' '/*.mat']);
colMat = get(gca,'colororder')';

trials = ["08"; "10"; "12"; "13"; "17"; "18"; "19"; "20"; "22"; "23"; "24"];
starts = [ 232;  255;  176;   64;  956;  173;  377;  418;  467;  770;    1];
offsets =[0.95;  0.9;    1;  0.7;  1.1; 1.07; 1.04;  1.3; 0.95; 0.92; 0.95];
ends =   [  -1;   -1;   -1;   -1;   -1;   -1;   -1; 1003;   -1;   -1;   -1];

RMS_errors = zeros(size(starts));

th0 = [];
nozAng = [];
color = 1;
inds = [3, 5, 7, 8, 9, 10, 11, 12, 14, 15];
for ind = 1:length(inds)
    load([fileNames(inds(ind)).folder, '/test', num2str(inds(ind))]);
    
    qB = EXP_theta.signals(1).values(:, 2);    
    tB = linspace(0, length(qB)/1000, length(qB));

    B = linResample(qB, 1000, 200);
    B = B(~isnan(B));
    t = (0:length(B) - 1) * 0.005;


    filename = append('../MoCap/CGT_on/block_121121_', trials(ind), '.c3d');

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
    
    [y1, x1] = max(B);
    [y2, x2] = max(q);

    B = B(x1-x2:end) - (y1 - y2);
    B = B(1:length(q))';

    plot(q);
    hold on; 
    plot(B, '--')

    RMS_errors(ind) = rms(B-q);
end

mean(RMS_errors)
% xlim([-12, 20]);
% ylim([-25, 60]);
% xlabel('Block Angle (deg)');
% ylabel('Block Vel (deg/s)');
% plot(qB(10000), control_modifiers.signals(3).values(10000), 'xr');
% plot(qFuture(10000), dqFuture(10000), 'or', 'LineWidth', 2)
% plot(qFuture(10000), control_modifiers.signals(3).values(10000), 'xr');
