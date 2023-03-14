import org.opensim.modeling.*

%% Create the springs
hipSpring = SpringGeneralizedForce('hip_rz');
legSpring = SpringGeneralizedForce('ankle_rz');

% Set names of the forces.
hipSpring.setName('spring_hip');
legSpring.setName('spring_ankle');

% Set the params
stiffness = 1000;
restLength = 0.005;
viscosity = 100;
hipSpring.setStiffness(stiffness);
hipSpring.setRestLength(restLength);
hipSpring.setViscosity(viscosity);

stiffness = 10;
restLength = 0.005;
viscosity = 100;
legSpring.setStiffness(stiffness);
legSpring.setRestLength(restLength);
legSpring.setViscosity(viscosity);

% Add the forces to the model
osimModel.addForce(hipSpring);
osimModel.addForce(legSpring);

%% Finalize connections
osimModel.finalizeConnections()
