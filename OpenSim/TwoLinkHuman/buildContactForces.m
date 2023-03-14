import org.opensim.modeling.*

% Section: Add HuntCrossleyForces
stiffness           = 10000000;
dissipation         = 2.0;
staticFriction      = 10;
dynamicFriction     = 0.7;
viscousFriction     = 0.7;
transitionVelocity  = 2;
 
% Make a Hunt Crossley Force for Pelvis and update parameters
PelvisHCF = HuntCrossleyForce();
PelvisHCF.setName('PelvisForce');
PelvisHCF.addGeometry('PelvisContact');
PelvisHCF.addGeometry('Ground_HalfSpace');
PelvisHCF.setStiffness(stiffness);
PelvisHCF.setDissipation(dissipation);
PelvisHCF.setStaticFriction(staticFriction);
PelvisHCF.setDynamicFriction(dynamicFriction);
PelvisHCF.setViscousFriction(viscousFriction);
PelvisHCF.setTransitionVelocity(transitionVelocity);
osimModel.addForce(PelvisHCF);
 
% Make a Hunt Crossley Force for Pelvis and update parameters
HeadHCF = HuntCrossleyForce();
HeadHCF.setName('HeadForce');
HeadHCF.addGeometry('HeadContact');
HeadHCF.addGeometry('Ground_HalfSpace');
HeadHCF.setStiffness(stiffness);
HeadHCF.setDissipation(dissipation);
HeadHCF.setStaticFriction(staticFriction);
HeadHCF.setDynamicFriction(dynamicFriction);
HeadHCF.setViscousFriction(viscousFriction);
HeadHCF.setTransitionVelocity(transitionVelocity);
osimModel.addForce(HeadHCF);

