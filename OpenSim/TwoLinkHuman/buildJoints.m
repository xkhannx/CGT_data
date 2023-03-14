import org.opensim.modeling.*
%% Leg to ground Pin joint
% Make and add a Pin joint for the Leg Body
locationInParent    = Vec3(0,0,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,-legLen/2,0);
orientationInChild  = Vec3(0,0,0);
legToGround    = PinJoint('LegToGround',...  % Joint Name
                                ground,...             % Parent Frame
                                locationInParent,...   % Translation in Parent Frame
                                orientationInParent,...% Orientation in Parent Frame
                                leg,...           % Child Frame
                                locationInChild,...    % Translation in Child Frame
                                orientationInChild);   % Orientation in Child Frame
 % TODO: Set the coordinate properties of the Pin Joint
leg_rz = legToGround.upd_coordinates(0);
leg_rz.setRange([deg2rad(-90), deg2rad(90)]);
leg_rz.setName('ankle_rz');
leg_rz.setDefaultValue(0);
leg_rz.setDefaultSpeedValue(0);

% Add Joint to model
osimModel.addJoint(legToGround)
%% Trunk bias Weld joint
% Make and add a Weld joint for the Trunk offset
locationInParent    = Vec3(0,legLen/2,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,0,0);
orientationInChild  = Vec3(0,0,deg2rad(trunk_leg_ang_offset));
hipOffset    = WeldJoint('hipOffset',...  % Joint Name
                                leg,...             % Parent Frame
                                locationInParent,...   % Translation in Parent Frame
                                orientationInParent,...% Orientation in Parent Frame
                                trunkBias,...           % Child Frame
                                locationInChild,...    % Translation in Child Frame
                                orientationInChild);   % Orientation in Child Frame

% Add Joint to model
osimModel.addJoint(hipOffset)

%% Head weld joint
% Make and add a Weld joint for the Head
locationInParent    = Vec3(0,hatLen / 2 + (0.936 - 0.818)*h,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,0,0);
orientationInChild  = Vec3(0,0,0);
headWeld    = WeldJoint('HeadWeld',...  % Joint Name
                                hat,...             % Parent Frame
                                locationInParent,...   % Translation in Parent Frame
                                orientationInParent,...% Orientation in Parent Frame
                                head,...           % Child Frame
                                locationInChild,...    % Translation in Child Frame
                                orientationInChild);   % Orientation in Child Frame

% Add Joint to model
osimModel.addJoint(headWeld)

%% Nozzle weld joint
% Make and add a Weld joint for the Head
locationInParent    = Vec3(0,hatLen / 2 - 0.02*h,0);
orientationInParent = Vec3(0,0,pi/2);
locationInChild     = Vec3(0,-0.1,0);
orientationInChild  = Vec3(0,0,0);
nozzleWeld    = WeldJoint('NozzleWeld',...  % Joint Name
                                hat,...             % Parent Frame
                                locationInParent,...   % Translation in Parent Frame
                                orientationInParent,...% Orientation in Parent Frame
                                nozzle,...           % Child Frame
                                locationInChild,...    % Translation in Child Frame
                                orientationInChild);   % Orientation in Child Frame

% Add Joint to model
osimModel.addJoint(nozzleWeld)







%% Make and add a Pin joint for the HAT Body
locationInParent    = Vec3(0,0,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,-hatLen/2,0);
orientationInChild  = Vec3(0,0,0);
hip    = PinJoint('Hip',...  % Joint Name
                                trunkBias,...             % Parent Frame
                                locationInParent,...   % Translation in Parent Frame
                                orientationInParent,...% Orientation in Parent Frame
                                hat,...           % Child Frame
                                locationInChild,...    % Translation in Child Frame
                                orientationInChild);   % Orientation in Child Frame
                            
 % TODO: Set the coordinate properties of the Pin Joint
hip_rz = hip.upd_coordinates(0);
hip_rz.setRange([deg2rad(-90), deg2rad(90)]);
hip_rz.setName('hip_rz');
hip_rz.setDefaultValue(0);
hip_rz.setDefaultSpeedValue(0);

% Add Joint to model
osimModel.addJoint(hip)


%% Hip Ang constraint
a = ArrayStr();
a.append('ankle_rz');
invAng = LinearFunction(-1, 0);

verTrunk = CoordinateCouplerConstraint();
verTrunk.setName('verTrunk');
verTrunk.setIndependentCoordinateNames(a);
verTrunk.setDependentCoordinateName('hip_rz');
verTrunk.setFunction(invAng);

osimModel.addConstraint(verTrunk);
