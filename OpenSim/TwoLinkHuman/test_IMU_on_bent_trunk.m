clear; clc; close all;

m = 50;
hPerson = 1.6;

top.m = 0.678 * m;
top.h = 0.47 * hPerson;

bottom.m = m - top.m;
bottom.h = hPerson - top.h;

comPerson = (top.m * (top.h/2 + bottom.h) + bottom.h/2 * bottom.m) / m;

initAsymBlock;

Rblock.m = m;

%%
trunks = 0:20;
th_act = trunks;
F_act = trunks;
F = F_act;
lens = F;

w0 = 10;
th0 = 15;

for ind = 1:length(trunks)
    % trunk offset
    trunkAng = trunks(ind);
    legAng = trunkAng + th0;
    
    % what cgt thinks
    Rblock.LBack = comPerson;
    F(ind) = max(min(asym_ansatz(0.74*trunkAng + th0, w0, Rblock, tank), tank.Fmax), 0);
    
    % actual resultant theta
    comBottom = [bottom.h / 2 * sind(legAng), bottom.h / 2 * cosd(legAng), 0];
    comTop = [bottom.h * sind(legAng) + top.h/2 * sind(th0), bottom.h * cosd(legAng) + top.h/2 * cosd(th0), 0];
    
    newCom = (bottom.m * comBottom + top.m * comTop) / Rblock.m;
    
    th_act(ind) = atan2d(newCom(1), newCom(2));
    Rblock.LBack = sqrt(newCom(1)^2 + newCom(2)^2);    
    F_act(ind) = max(min(asym_ansatz(th_act(ind), w0, Rblock, tank), tank.Fmax), 0);
    
    lens(ind) = Rblock.LBack;
end

% plot(trunks, lens - comPerson)
plot(trunks, [th_act; 0.74 * trunks + th0])
% plot(trunks, [F_act; F]')

%%
% ths = 0:25;
% th_act = ths;
% F_act = ths;
% F = F_act;
% 
% w0 = 0;
% th0 = 0;
% trunkAng = 1;
% 
% for ind = 1:length(ths)
%     th0 = ths(ind);
%     % what cgt thinks
%     Rblock.LBack = comPerson;
%     F(ind) = max(min(asym_ansatz(th0, w0, Rblock, tank), tank.Fmax), 0);
%     
%     % trunk offset
%     legAng = trunkAng + th0;
%     
%     comBottom = [bottom.h / 2 * sind(legAng), bottom.h / 2 * cosd(legAng), 0];
%     comTop = [bottom.h * sind(legAng) + top.h/2 * sind(th0), bottom.h * cosd(legAng) + top.h/2 * cosd(th0), 0];
%     
%     newCom = (bottom.m * comBottom + top.m * comTop) / Rblock.m;
%     
%     th_act(ind) = atan2d(newCom(1), newCom(2));
%     Rblock.LBack = sqrt(newCom(1)^2 + newCom(2)^2);    
%     F_act(ind) = max(min(asym_ansatz(th_act(ind), w0, Rblock, tank), tank.Fmax), 0);
%     
% end
% 
% plot([ths; th_act]')
% plot(ths, [F-F_act]')