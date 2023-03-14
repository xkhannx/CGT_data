import org.opensim.modeling.*

h = 1.6;
m = 52;
% Parameters
hatMass = 0.678 * m;
hatLen = (0.818 - 0.53) * h;
hatIcom = 1.8;

% Make and add a HAT Body
hat = Body();
hat.setName('HAT');
hat.setMass(hatMass);
hat.setInertia( Inertia(1,1,hatIcom,0,0,0) );

% Add geometry to the body
hatGeometry = Brick(Vec3(0.1, hatLen / 2, 0.15));
hatGeometry.setColor( Vec3(0.2, 0.6, 0.9) );
hat.attachGeometry(hatGeometry);

% Add Body to the Model
osimModel.addBody(hat);

% Add leg
legMass = m - hatMass;
legLen = 0.53 * h;
legIcom = 0.326^2 * legMass;

% Make and add a HAT Body
leg = Body();
leg.setName('Legs');
leg.setMass(legMass);
leg.setInertia( Inertia(1,1,legIcom,0,0,0) );

% Add geometry to the body
legGeometry = Brick(Vec3(0.075, legLen / 2, 0.075));
legGeometry.setColor( Vec3(0.2, 0.9, 0.7) );
leg.attachGeometry(legGeometry);

% Add Body to the Model
osimModel.addBody(leg);

% Add hip offset
trunkBias = Body();
trunkBias.setName('Trunk_bias');
trunkBias.setMass(0);

osimModel.addBody(trunkBias);

% Add head 
head = Body();
head.setName('Head');
head.setMass(0);

osimModel.addBody(head);

% Add NOZZLE
nozzle = Body();
nozzle.setName('Nozzle');
nozzle.setMass(0);
% Add geometry to the body
nozzleGeometry = Cylinder(0.02, 0.1);
nozzleGeometry.setColor( Vec3(0.8, 0.5, 0.2) );
nozzle.attachGeometry(nozzleGeometry);

osimModel.addBody(nozzle);
