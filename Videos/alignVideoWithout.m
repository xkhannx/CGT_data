clear; close all; clc;
ind = 10;

videoTimeOffsetInds = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1];
viconTimeOffsetInds = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1];
yOffsets = [0.7, 0.65, 0, 0.8512, 0, 0, 0, 0.76, 0, 0.7365];

starts = [   1;  527;  486;  555;  470;  325;  576;  640;  455;  420];
trials = ["01"; "02"; "03"; "04"; "06"; "08"; "10"; "11"; "19"; "20"];

%%
fileID = fopen('Frames/without/without_'+ trials(ind) + '.txt','r');
A = fscanf(fileID,'%f');
fclose(fileID);

t = 0:(1/30):((length(A)-1)/30);
trials(ind)
[xx, yy] = max(A)
offsetVideoData = A + yOffsets(ind);
plot(t - t(videoTimeOffsetInds(ind)), offsetVideoData)
grid on; hold on
plot(t(1) - t(videoTimeOffsetInds(ind)) * ones(2, 1), [-10, 15])
%%
colMat = get(gca,'colororder');

ends =   [  -1;  975;   -1;   -1;   -1;   -1;  845;  690;   -1;   -1];
offsets =[   1;    1;    1; 0.78;    0;    0; 0.88;    1;  0.6;  0.6];

filename = append('block_noThrust_121521/block_noThrust_121521_', trials(ind), '.c3d');

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

[xx, yy] = max(q)

plot(t - t(viconTimeOffsetInds(ind)), q, 'Color', [colMat(1, :) 0.5]);
%%
% figure(2)
% plot(q, 'Color', [colMat(1, :) 0.5]);
% hold on; grid on;
% plot(A)