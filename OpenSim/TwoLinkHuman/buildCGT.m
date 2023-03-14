import org.opensim.modeling.*

% cgt = PointActuator('Nozzle');
% cgt.setName('cgt');
% 
% controller = PrescribedController();
% controller.addActuator(cgt);
% controller.prescribeControlForActuator('cgt', StepFunction(0.5, 1.2, 0, 360));
% %%
% osimModel.addForce(cgt);
% %%
% osimModel.addController(controller);
%%
cgtFun = @(t) 338.4456 * exp(-t / 0.1467);

t = (0:0.1:1)';
x = -cgtFun(t) * 0.9;

thrust = PiecewiseLinearFunction;
for ind = 1:length(t)
    thrust.addPoint(t(ind), x(ind));
end
EFF = PiecewiseLinearFunction;
for ind = 1:length(t)
    EFF.addPoint(t(ind), 0);
end

cgt = PrescribedForce('cgt', nozzle);
cgt.setPointFunctions(EFF, EFF, EFF);
cgt.setForceIsInGlobalFrame(false);
cgt.setForceFunctions(EFF, thrust, EFF);


osimModel.addForce(cgt);
