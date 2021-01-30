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
	void setStiffness(float stiffness);
	void setDampingFactor(float damping); // Note that only springs are damped
	int addRigidBody(Vec3 position, Vec3 size, double mass); // size in {x, y, z}
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

	float invScale = 9;
	int xSize = 9;
	int ySize = 9;
	float offsetX = -1 * floor(xSize / 2);
	float offsetY = -1 * floor(ySize / 2);
	float box_size = 0.005;
	float mass = 0.5;

	// Methods
};
#endif