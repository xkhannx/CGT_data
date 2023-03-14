clear; close all; clc;
ind = 10;

videoTimeOffsetInds = [30, 10, 6, 8, 9, 16, 10, 17, 12, 23];
viconTimeOffsetInds = [133, 62, 34, 47, 54, 100, 60, 107, 74, 147];
fires  = [26, 5, 1, 3, 6, 12, 5, 9, 4, 19];

trials = ["08"; "10"; "12"; "13"; "17"; "18"; "19"; "20"; "22"; "23"; "24"];
starts = [ 232;  255;  176;   64;  956;  173;  377;  418;  467;  770;    1];

%%
fileID = fopen('Frames/with/with_'+ trials(ind) + '.txt','r');
A = fscanf(fileID,'%f');
fclose(fileID);

t = 0:(1/30):((length(A)-1)/30);

[xx, yy] = max(A)
plot(t - t(videoTimeOffsetInds(ind)), A)
grid on; hold on
plot(t(fires(ind)) - t(videoTimeOffsetInds(ind)) * ones(2, 1), [-10, 15])
%%
colMat = get(gca,'colororder');

offsets =[0.95;  0.9;    1;  0.7;  1.1; 1.07; 1.04;  1.3; 0.95; 0.92; 0.95];
ends =   [  -1;   -1;   -1;   -1;   -1;   -1;   -1; 1003;   -1;   -1;   -1];

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
q((1113+starts(ind)):(1194 + starts(ind))) = q((1113+starts(ind)):(1194 + starts(ind))) + 0.3449;

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

figure(2)
plot(q, 'Color', [colMat(1, :) 0.5]);
hold on; grid on;
plot(A)