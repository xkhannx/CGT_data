import org.opensim.modeling.*

% Make a Contact Half Space
groundContactLocation = Vec3(0,0,0);
groundContactOrientation = Vec3(0,0,-pi/2);
groundContactSpace = ContactHalfSpace(groundContactLocation,...
                                       groundContactOrientation,...
                                       ground);
groundContactSpace.setName('Ground_HalfSpace');
osimModel.addContactGeometry(groundContactSpace);

%% Make a Pelvis Contact
contactSphereRadius = 0.1;

PelvisContactSphere = ContactSphere();
PelvisContactSphere.setRadius(contactSphereRadius);
PelvisContactSphere.setLocation( Vec3(0, legLen/2, 0) );
PelvisContactSphere.setFrame(leg)
PelvisContactSphere.setName('PelvisContact');
osimModel.addContactGeometry(PelvisContactSphere);

%% Make a Head Contact
contactSphereRadius = 0.1;

HeadContactSphere = ContactSphere();
HeadContactSphere.setRadius(contactSphereRadius);
HeadContactSphere.setLocation( Vec3(0, 0, 0) );
HeadContactSphere.setFrame(head)
HeadContactSphere.setName('HeadContact');
osimModel.addContactGeometry(HeadContactSphere);
