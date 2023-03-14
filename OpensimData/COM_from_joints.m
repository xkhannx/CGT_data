function [comAng, rCom] = COM_from_joints(ank, hip)
a1 = ank;
a2 = 0;
a3 = hip;

mFoot = 2 * 0.0145;
mShank = 2 * 0.0465;
mThigh = 2 * 0.1;
mHat = 0.678;

L1 = 0.246;
L2 = 0.245;
L3 = 0.288;

x1 = 0.567 * L1 * sind(a1);
x2 = L1 * sind(a1) + 0.567 * L2 * sind(a1 + a2);
x3 = L1 * sind(a1) + L2 * sind(a1 + a2) + 0.626 * L3 * sind(a1 + a2 + a3);

y1 = 0.039 + 0.567 * L1 * cosd(a1);
y2 = 0.039 + L1 * cosd(a1) + 0.567 * L2 * cosd(a1 + a2);
y3 = 0.039 + L1 * cosd(a1) + L2 * cosd(a1 + a2) + 0.626 * L3 * cosd(a1 + a2 + a3);

xCom = (mShank * x1 + mThigh * x2 + mHat * x3) / (mShank + mThigh + mHat);
yCom = (mFoot * 0.039/2 + mShank * y1 + mThigh * y2 + mHat * y3) / (mFoot + mShank + mThigh + mHat);
rCom = sqrt(xCom .^ 2 + yCom .^ 2) * 1.55;

comAng = atan2d(xCom, yCom);
end