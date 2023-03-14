clear; close all; clc;

%% Import OpenSim Libraries into Matlab
import org.opensim.modeling.*

%% Instantiate an (empty) OpenSim Model
osimModel = Model();
osimModel.setName('TwoLink');

% Get a reference to the ground object
ground = osimModel.getGround();

% Define the acceleration of gravity
osimModel.setGravity(Vec3(0, -9.80665, 0));

%% Add bodies
trunk_leg_ang_offset = 0;

buildBodies;
buildJoints;

buildCGT;

buildContactPoints;
buildContactForces;
buildSpringForces;

%% Initialize the System (checks model consistency).
osimModel.finalizeConnections();
osimModel.initSystem();

% Save the model to a file
osimModel.print('Model/TwoLink.osim');
disp('TwoLink.osim printed!');
