#ifndef MSSRBSIMULATOR_h
#define MSSRBSIMULATOR_h
#include "MassSpringSystemSimulator.h"
#include "RigidBodySystemSimulator.h"

class MSSRBSimulator:public Simulator{
public:
	// Construtors
	MSSRBSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	const char* getIntegratorsStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setGravity(Vec3 g);
    void setMass(float mass); // MassSpringSystemSimulator does not support assigning different masses to different objects.
	void setStiffness(float stiffness);
	void setDampingFactor(float damping); // Note that only springs are damped
	int addRigidBody(Vec3 position, Vec3 size); // size in {x, y, z}
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);
	void setAngularVelocityOf(int i, Vec3 ang_vel);
	void addSpring(int masspoint1, int masspoint2, float initialLength);

	//Tweakbar's Callbacks
	static void TW_CALL GetGravityCallback(void* value, void* clientData);
	static void TW_CALL SetGravityCallback(const void* value, void* clientData);

	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	int m_iIntegrator=EULER;
	MassSpringSystemSimulator mss;
	RigidBodySystemSimulator rb;

	// UI Attributes

	//For 1st result print
	bool isFirst = true;

	// Methods
};
#endif