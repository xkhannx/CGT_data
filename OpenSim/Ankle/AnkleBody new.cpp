#include <OpenSim/OpenSim.h>
#include "OpenSim/Common/STOFileAdapter.h"
#include "OpenSim/Common/Object.h"
#include <ctime>    // for clock()

using namespace OpenSim;
using namespace SimTK;
using namespace std;

class JointController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(JointController, Controller);
    // This section contains methods that can be called in this controller class.
public:
    /**
     * Constructor
     *
     * @param aModel Model to be controlled
     * @param aKp Position gain by which the position error will be multiplied
     */
    JointController(double aKp, double aKv) : Controller(), kp(aKp), kv(aKv)
    {
    }

    /**
     * This function is called at every time step for every actuator.
     *
     * @param s Current state of the system
     * @param controls Controls being calculated
     */
    void computeControls(const SimTK::State& s, SimTK::Vector& controls) const
    {
        bool pidON = false;
        double maxAnkleTorque = 90, maxHipTorque = 130;
        // Get the current time in the simulation.
        double t = s.getTime();
        if (t > 0.1) { pidON = true; }

        // Read the mass of the block.
        //double blockMass = getModel().getBodySet().get("block").getMass();

        ///////////////////////////////////////////
        // ANKLE CONTROL
        ///////////////////////////////////////////
        TorqueActuator* ankleTorque = dynamic_cast<TorqueActuator*>(&getModel().getActuators().get(0));
        const Coordinate& ankleCoord = _model->getCoordinateSet().get("ankleAng");

        double ankleAng = ankleCoord.getValue(s);
        double ankleVel = ankleCoord.getSpeedValue(s);

        
        double ankleTorqueMag = 0.0;
        if (pidON) {
            if (ankleAng <= 4.0 / 180.0 * Pi && ankleAng >= -8.0 / 180.0 * Pi) {
                ankleTorqueMag = kp * ankleAng + kv * ankleVel;
            }
        }
        else {
            // PASSIVE TORQUE!!!!!!!!!!!
            //ankleTorqueMag = exp(2.1016 - 0.0843 * ankleAng * 180.0 / Pi) - exp(-7.9763 + 0.1949 * ankleAng * 180.0 / Pi) - 1.792;
            ankleTorqueMag = 190 * ankleAng + 3 * ankleVel;
        }

        if (abs(ankleTorqueMag) > maxAnkleTorque) {
            ankleTorqueMag = sign(ankleTorqueMag) * maxAnkleTorque;
        }
        
        Vector ankleControl(1, ankleTorqueMag);
        // Add in the controls computed for this muscle to the set of all model controls
        ankleTorque->addInControls(ankleControl, controls);

        ///////////////////////////////////////////
        // HIP CONTROL
        ///////////////////////////////////////////
        TorqueActuator* hipTorque = dynamic_cast<TorqueActuator*>(&getModel().getActuators().get(1));
        const Coordinate& hipCoord = _model->getCoordinateSet().get("hipAng");

        double hipAng = hipCoord.getValue(s);
        double hipVel = hipCoord.getSpeedValue(s);

        double hipTorqueMag = 0; 
        if (pidON) {
            hipTorqueMag = kp * hipAng + kv * hipVel; // straight body
            //hipTorqueMag = kp * (hipAng + 20 * Pi/180) + kv * hipVel; // vertical trunk
        }
        else {
            // PASSIVE TORQUE!!!!!!!!!!!
            //hipTorqueMag = exp(1.4655 - 0.075 * hipAng * 180.0 / Pi) - exp(1.3403 + 0.0305 * hipAng * 180.0 / Pi) + 8.072;
            hipTorqueMag = 240 * hipAng + 40 * hipVel;
            //hipTorqueMag = 50 * (hipAng + 20 * Pi / 180) + 40 * hipVel;
        }

        if (abs(hipTorqueMag) > maxHipTorque) {
            hipTorqueMag = sign(hipTorqueMag) * maxHipTorque;
        }

        Vector hipControl(1, hipTorqueMag);
        // Add in the controls computed for this muscle to the set of all model controls
        hipTorque->addInControls(hipControl, controls);
    }
    
    // This section contains the member variables of this controller class.
private:

    /** Position gain for this controller */
    double kp;
    /** Velocity gain for this controller */
    double kv;

};


int main()
{
    try {
        bool cgtON = false;
        double nozAng = 0;
        double initAnkleAng = 0; // straight body - 17.1, vertical trunk - 21
        double initAnkleVel = 20; //41;

        double initHipAng = 0; // straight body
        //double initHipAng = -initAnkleAng; // vertical trunk
        double initHipVel = 0;

        // Set gain for the controller.
        double kp = 600.0; // position gain
        double kv = 80.0;   // velocity gain
        //////////////////////
        // MODEL PARAMETERS //
        //////////////////////
        double h = 1.54;
        double m = 52;
        
        // Create an OpenSim model and set its name
        Model osimModel;
        osimModel.setName("AnkleBody");
        // GROUND FRAME
        // Get a reference to the model's ground frame
        Ground& ground = osimModel.updGround();

        ///////////////////////////////////////////
        // DEFINE BODIES AND JOINTS OF THE MODEL //
        ///////////////////////////////////////////
        
        ///////////////////////////////////////////
        // Shank body
        double shankMassX2 = (0.0465 + 0.0145) * m * 2;
        Vec3 shankSize(0.1, (0.246 + 0.039) * h, 0.2);
        double rGyr = 0.302 * shankSize(1);
        Inertia shankInertia = Inertia(1, 1, shankMassX2 * rGyr * rGyr);
        OpenSim::Body* shankBody = new OpenSim::Body("Shanks", shankMassX2, Vec3(0), shankInertia);

        // Shank geometry 
        Sphere* shankCOMSphere = new Sphere(0.05);
        shankCOMSphere->setName("shankCOM");
        shankCOMSphere->upd_Appearance().set_color(Vec3(1, 0, 0.0));
        shankBody->attachGeometry(shankCOMSphere);

        osimModel.addBody(shankBody);

        PhysicalFrame* shankOrigin = new PhysicalOffsetFrame(*shankBody, Transform(Vec3(0, -0.067* shankSize(1), 0)));
        shankOrigin->setName("ShanksGeometricCenter");

        Brick* shankGeometry = new Brick(shankSize / 2);
        shankGeometry->upd_Appearance().set_color(Vec3(0.25, 1.0, 0.0));
        shankGeometry->upd_Appearance().set_opacity(0.5);
        shankOrigin->attachGeometry(shankGeometry);

        osimModel.addComponent(shankOrigin);


        // Pin joint Shank to Ground
        Vec3 locationInParent(0);
        Vec3 locationInChild(0, -shankSize(1) * 0.567, 0);
        Vec3 orientationInParent(0);
        Vec3 orientationInChild(0);

        PinJoint* ankleJoint = new PinJoint("ankle",
            ground, locationInParent, orientationInParent,
            *shankBody, locationInChild, orientationInChild);
        ankleJoint->updCoordinate().setName("ankleAng");
                
        // Limit the range of motion for the hip and knee joints.
 /*       auto& ankleCoord = ankleJoint->updCoordinate(PinJoint::Coord::RotationZ);
        ankleCoord.setDefaultValue(0);

        double ankleRange[2] = { 90., -90. };
        double ankleStiff[2] = { 200000., 200000. }, ankleDamping = 50000., ankleTransition = 10.;
        auto ankleLimitForce = new CoordinateLimitForce("ankleAng", ankleRange[0],
            ankleStiff[0], ankleRange[1], ankleStiff[1], ankleDamping, ankleTransition);
        ankleJoint->addComponent(ankleLimitForce);*/

        osimModel.addJoint(ankleJoint);
        ///////////////////////////////////////////
        // Thigh body
        double thighMassX2 = 0.1 * m * 2;
        Vec3 thighSize(0.1, 0.245 * h, 0.191 * h);
        rGyr = 0.323 * thighSize(1);
        Inertia thighInertia = Inertia(1, 1, thighMassX2 * rGyr * rGyr);
        OpenSim::Body* thighBody = new OpenSim::Body("Thighs", thighMassX2, Vec3(0), thighInertia);

        // Thigh geometry 
        Sphere* thighCOMSphere = new Sphere(0.05);
        thighCOMSphere->setName("thighCOM");
        thighCOMSphere->upd_Appearance().set_color(Vec3(1, 0, 0.0));
        thighBody->attachGeometry(thighCOMSphere);

        osimModel.addBody(thighBody);

        PhysicalFrame* thighOrigin = new PhysicalOffsetFrame(*thighBody, Transform(Vec3(0, -0.067 * thighSize(1), 0)));
        thighOrigin->setName("ThighsGeometricCenter");

        Brick* thighGeometry = new Brick(thighSize / 2);
        thighGeometry->upd_Appearance().set_color(Vec3(0.5, 1.0, 0.0));
        thighGeometry->upd_Appearance().set_opacity(0.5);
        thighOrigin->attachGeometry(thighGeometry);

        osimModel.addComponent(thighOrigin);

        // Knee joint Thigh to Shank
        locationInParent.set(0, 0);
        locationInParent.set(1, shankSize(1) * 0.433);
        locationInParent.set(2, 0);
        locationInChild.set(0, 0);
        locationInChild.set(1, -thighSize(1) * 0.567);
        locationInChild.set(2, 0);

        WeldJoint* kneeJoint = new WeldJoint("kneeWeld",
            *shankBody, locationInParent, orientationInParent,
            *thighBody, locationInChild, orientationInChild);
        /*
        PinJoint* kneeJoint = new PinJoint("kneeJoint",
            *shankBody, locationInParent, orientationInParent,
            *thighBody, locationInChild, orientationInChild);
        kneeJoint->updCoordinate().setName("kneeAng");
*/
        osimModel.addJoint(kneeJoint);
        

        // HAT body
        double hatMass = 0.678 * m;
        Vec3 hatSize(0.1, 0.288 * h, 0.259 * h);
        rGyr = 0.496 * hatSize(1);
        Inertia hatInertia = Inertia(1, 1, hatMass * rGyr * rGyr);
        OpenSim::Body* hatBody = new OpenSim::Body("HAT", hatMass, Vec3(0), hatInertia);

        // HAT geometry 
        Sphere* hatCOMSphere = new Sphere(0.05);
        hatCOMSphere->setName("hatCOM");
        hatCOMSphere->upd_Appearance().set_color(Vec3(1, 0, 0.0));
        hatBody->attachGeometry(hatCOMSphere);

        osimModel.addBody(hatBody);

        PhysicalFrame* hatOrigin = new PhysicalOffsetFrame(*hatBody, Transform(Vec3(0, -0.126 * hatSize(1), 0)));
        hatOrigin->setName("HATGeometricCenter");

        Brick* hatGeometry = new Brick(hatSize / 2);
        hatGeometry->upd_Appearance().set_color(Vec3(0.5, 1.0, 0.0));
        hatGeometry->upd_Appearance().set_opacity(0.5);
        hatOrigin->attachGeometry(hatGeometry);

        osimModel.addComponent(hatOrigin);

        // Weld joint HAT to Thigh
        locationInParent.set(0, 0);
        locationInParent.set(1, thighSize(1) * 0.433);
        locationInParent.set(2, 0);
        locationInChild.set(0, 0);
        locationInChild.set(1, -hatSize(1) * 0.626);
        locationInChild.set(2, 0);

        //WeldJoint* hipJoint = new WeldJoint("hipWeld",
        //    *thighBody, locationInParent, orientationInParent,
        //    *hatBody, locationInChild, orientationInChild);
        PinJoint* hipJoint = new PinJoint("hipJoint",
            *thighBody, locationInParent, orientationInParent,
            *hatBody, locationInChild, orientationInChild);
        hipJoint->updCoordinate().setName("hipAng");

        osimModel.addJoint(hipJoint);
        
        ///////////////////////////////////////////
        // CONTACT GEOMETRY
        ///////////////////////////////////////////
        ContactHalfSpace* floor = new ContactHalfSpace(Vec3(0), Vec3(0, 0, -Pi / 2.), ground, "floor");
        osimModel.addContactGeometry(floor);

        double stiffness = 1.e8, dissipation = 0.5, friction[3] = { 0.9, 0.9, 0.6 };

        //// Pelvis contact
        auto pelvis = new ContactSphere(0.1, Vec3(0, thighSize(1) * 0.433, 0), *thighBody, "pelvis");
        osimModel.addContactGeometry(pelvis);

        auto contactParams = new OpenSim::HuntCrossleyForce::ContactParameters(stiffness,
            dissipation, friction[0], friction[1], friction[2]);
        contactParams->addGeometry("floor");
        contactParams->addGeometry("pelvis"); 

        auto contactForce = new OpenSim::HuntCrossleyForce(contactParams);
        contactForce->setName("Pelvis_contact_force");
        osimModel.addForce(contactForce);

        // Head contact
        auto head = new ContactSphere(0.1, Vec3(0, hatSize(1) * 0.374 + h * 0.118, 0), *hatBody, "head");
        osimModel.addContactGeometry(head);

        auto headContactParams = new OpenSim::HuntCrossleyForce::ContactParameters(stiffness,
            dissipation, friction[0], friction[1], friction[2]);
        headContactParams->addGeometry("floor");
        headContactParams->addGeometry("head");
        osimModel.addForce(new OpenSim::HuntCrossleyForce(headContactParams));


        ///////////////////////////////////////////
        // ACTUATORS
        ///////////////////////////////////////////

        TorqueActuator* ankleTorque = new TorqueActuator(ground, *shankBody, Vec3(0, 0, 1), true);
        ankleTorque->setName("ankleTorque");
        ankleTorque->setOptimalForce(1);
        osimModel.addForce(ankleTorque);

        TorqueActuator* hipTorque = new TorqueActuator(*thighBody, *hatBody, Vec3(0, 0, 1), true);
        hipTorque->setName("hipTorque");
        hipTorque->setOptimalForce(1);
        osimModel.addForce(hipTorque);
        //BushingForce* kneeTorque = new BushingForce("kneeBushing", *shankBody, *thighBody);
        //kneeTorque->set_rotational_stiffness(Vec3(0, 0, 100));
        //kneeTorque->set_rotational_damping(Vec3(0, 0, 100));
        //osimModel.addForce(kneeTorque);
        

        ///////////////////////////////////////////
        // CGT Force
        ///////////////////////////////////////////
        // the prescribed controller sets the controls as functions of time
        // Nozzle body
        Vec3 nozzleSize(0.5, 0.1, 0.1);
        OpenSim::Body* nozzleBody = new OpenSim::Body("Nozzle", 0, Vec3(0), Inertia(0));

        // Nozzle geometry 
        Brick* nozzleGeometry = new Brick(nozzleSize / 2);
        nozzleGeometry->upd_Appearance().set_color(Vec3(1.0, 0.0, 0.0));
        nozzleBody->attachGeometry(nozzleGeometry);

        osimModel.addBody(nozzleBody);

        // Weld joint HAT to nozzle
        locationInParent.set(0, 0);
        locationInParent.set(1, 0.75 * h - shankSize(1) - thighSize(1) - hatSize(1) * 0.626);
        locationInParent.set(2, 0);
        locationInChild.set(0, nozzleSize(0)/2);
        locationInChild.set(1, 0);
        locationInChild.set(2, 0);
        orientationInChild.set(2, nozAng * Pi/180);
        WeldJoint* nozzleJoint = new WeldJoint("nozzleWeld",
            *hatBody, locationInParent, orientationInParent,
            *nozzleBody, locationInChild, orientationInChild);

        osimModel.addJoint(nozzleJoint);

        // Linear actuator
        PointActuator* nozzleForce = new PointActuator("Nozzle");
        nozzleForce->setName("NozzleForce");
        nozzleForce->setOptimalForce(1.0);
        nozzleForce->set_point_is_global(false);
        nozzleForce->set_force_is_global(false);
        nozzleForce->set_direction(Vec3(1.0, 0.0, 0.0));

        osimModel.addForce(nozzleForce);

        PrescribedController* cgtController = new PrescribedController();
        cgtController->addActuator(*nozzleForce);

        const int numSamples = 1000;

        double t[numSamples+1];
        double x[numSamples+1];

        for (int i = 0; i < numSamples; i++)
        {
            t[i] = (double)i / numSamples;
            x[i] = 338.4456 * exp(-t[i] / 0.1467);
        }
        t[numSamples] = 10;
        x[numSamples] = 0;

        cgtController->prescribeControlForActuator("NozzleForce", new PiecewiseLinearFunction(numSamples + 1, t, x));
        cgtController->setEnabled(cgtON);

        osimModel.addController(cgtController);

        ///////////////////////////////////////////
        // To print (serialize) the latest connections of the model, it is
        // necessary to finalizeConnections() first.
        osimModel.finalizeConnections();
        // Save the model to a file
        if (cgtON) {
            osimModel.print("../Results/with/AnkleBody.osim");
        }
        else {
            osimModel.print("../Results/without/AnkleBody.osim");
        }
        


        ///////////////////////////////////////////
        // SIMULATION
        ///////////////////////////////////////////
        osimModel.setUseVisualizer(false);
        
        // CONTROLLERS

        // Print the control gains and block mass.
        std::cout << std::endl;
        std::cout << "kp = " << kp << std::endl;
        std::cout << "kv = " << kv << std::endl;

        // Create the controller.
        JointController* controller = new JointController(kp, kv);

        // Give the controller the Model's actuators so it knows
        // to control those actuators.
        controller->setActuators(osimModel.updActuators());

        // Add the controller to the Model.
        osimModel.addController(controller);

        // Define the initial and final simulation times.
        double initialTime = 0.0;
        double finalTime = 10.0;


        SimTK::State& si = osimModel.initSystem();

        cout << "Mass: " << osimModel.getTotalMass(si) << endl;
        cout << "COM position: " << osimModel.calcMassCenterPosition(si) << endl;
        cout << "Inertia about COM: " << osimModel.getInertiaAboutMassCenter(si) << endl;

        // Define non-zero (defaults are 0) states.
        CoordinateSet& modelCoordinateSet = osimModel.updCoordinateSet();
        Coordinate& ankleAng = modelCoordinateSet.get("ankleAng");
        ankleAng.setValue(si, initAnkleAng / 180.0 * Pi);
        ankleAng.setSpeedValue(si, initAnkleVel / 180.0 * Pi);

        Coordinate& hipAng = modelCoordinateSet.get("hipAng");
        hipAng.setValue(si, initHipAng / 180.0 * Pi);
        hipAng.setSpeedValue(si, initHipVel / 180.0 * Pi);

        Manager manager(osimModel);
        manager.setIntegratorAccuracy(1.0e-6);

        // Examine the model.
        osimModel.printDetailedInfo(si, std::cout);

        // Print out the initial position and velocity states.
        for (int i = 0; i < modelCoordinateSet.getSize(); i++) {
            std::cout << "Initial " << modelCoordinateSet[i].getName()
                << " = " << modelCoordinateSet[i].getValue(si)
                << ", and speed = "
                << modelCoordinateSet[i].getSpeedValue(si) << std::endl;
        }
        cout << "COM position: " << osimModel.calcMassCenterPosition(si) << endl;

        // Integrate from initial time to final time.
        si.setTime(initialTime);
        manager.initialize(si);
        cout << "\nIntegrating from " << initialTime << " to " << finalTime << endl;
        manager.integrate(finalTime);

        // Save the simulation results.
        auto controlsTable = osimModel.getControlsTable();

        auto statesTable = manager.getStatesTable();
        if (cgtON) {
            STOFileAdapter::write(controlsTable, "../Results/with/ankleBody_controls.sto");
            STOFileAdapter::write(statesTable, "../Results/with/ankleBody_states.sto");
        }
        else {
            STOFileAdapter::write(controlsTable, "../Results/without/ankleBody_controls.sto");
            STOFileAdapter::write(statesTable, "../Results/without/ankleBody_states.sto");
        }
    }

    catch(const OpenSim::Exception& ex)
    {
        std::cout << ex.getMessage() << std::endl;
        return 1;
    }
    catch(const std::exception& ex)
    {
        std::cout << ex.what() << std::endl;
        return 2;
    }
    catch(...)
    {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
        return 3;
    }

    std::cout << "OpenSim example completed successfully.\n";
    std::cout << "shit.\n";

    std::cin.get();
    return 0;
}