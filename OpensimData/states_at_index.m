clc

% ind = fire(1) + startInds(1) - 1
comAngThreshold = find(com_ang > 7.2);
ind = comAngThreshold(1)
com_ang(ind)

percent = 0;

ank1 = stateData.data(ind, 2)*180/pi;
ank2 = stateData.data(ind+1, 2)*180/pi;
ank = ank1 + (ank2 - ank1) * percent

ankVel1 = stateData.data(ind, 3)*180/pi;
ankVel2 = stateData.data(ind+1, 3)*180/pi;
ankVel = ankVel1 + (ankVel2 - ankVel1) * percent

hip1 = stateData.data(ind, 4)*180/pi;
hip2 = stateData.data(ind+1, 4)*180/pi;
hip = hip1 + (hip2 - hip1) * percent

hipVel1 = stateData.data(ind, 5)*180/pi;
hipVel2 = stateData.data(ind+1, 5)*180/pi;
hipVel = hipVel1 + (hipVel2 - hipVel1) * percent

comAngMid = com_ang(ind) + (com_ang(ind+1) - com_ang(ind)) * percent;
comVelMid = com_angvel(ind) + (com_angvel(ind+1) - com_angvel(ind)) * percent;
-(ank + hip - comAngMid)
plot(comAngMid, comVelMid, 'x');
%%
% hip = 0:40;
% 
% for ank = [0, 7, 13]
%     trunk = ank-hip;
%     plot(hip, [trunk; COM_from_joints(ank, -hip); ank*ones(size(trunk))], 'LineWidth', 5.1 - 5*ank/15); hold on; grid on;
% end

%% 
% ank = 0:40;
% plot(ank, [COM_from_joints(ank, -ank); 30 * ank/40]);