#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

class Spring {
public:
	int    point1;
	int    point2;
	float  initialLength;
};

class Point {
public:
	Vec3  position;
	Vec3  velocity;
	Vec3  force;
	Vec3 size;
	bool  isFixed;
};


class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	const char* getIntegratorsStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawSprings(ID3D11DeviceContext* pd3dImmediateContext);
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	int addMassPoint(Vec3 position, Vec3 Velocity, Vec3 size, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);

	//Tweakbar's Callbacks
	static void TW_CALL GetGravityCallback(void* value, void* clientData);
	static void TW_CALL SetGravityCallback(const void* value, void* clientData);

	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

public:
//private:
	// Data Attributes
	float m_fMass=1;
	float m_fStiffness=1;
	float m_fDamping=1;
	int m_iIntegrator=EULER;
	std::vector<Point> m_vPoints;
	std::vector<Spring> m_vSprings;
	float m_fFloor = -0.9;

	// UI Attributes
	Vec3 m_externalForce;
	Vec3 m_mouseForce;
	Point2D m_mouse{0};
	Point2D m_trackmouse{0};
	Point2D m_oldtrackmouse{0};

	float externalForce[3] = { 0,0,0 };

	//For 1st result print
	bool isFirst = true;

	// Methods
	void computeForces();
	Vec3 computeElasticForce(Spring s);
	void integrate(float ts);
	void integrateMidpoint(float ts);
	void integratePositionsEuler(float ts);
	void integrateVelocitiesEuler(float ts);
	void resolveCollisions();
	Vec3 getPositionOfMassPoint(int index);
};
#endif