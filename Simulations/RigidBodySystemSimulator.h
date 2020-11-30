#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "util/dxm_support.h"
#include "collisionDetect.h"

#define TESTCASEUSEDTORUNTEST 2

class Body {
public:
	bool isMovable;
	Vec3 pos;
	Vec3 vel;
	Vec3 force;
	Vec3 torque;
	Vec3 ang_mom;
	Vec3 ang_vel;
	Quat ang_pos{0,0,0,1};
	const Vec3 size;
	const double inverse_mass;
	const XMFLOAT3X3 inverse_inertia;

	Body(Vec3 pos, Vec3 size, double mass) :
		pos(pos),
		size(size),
		inverse_mass(1 / mass),
		inverse_inertia(inverse(getCuboidIntertia(size) * mass)),
		isMovable(mass != INFINITY)
	{}
	XMFLOAT3X3 getRotatedInverseIntertia() const {
		auto r4x4 = ang_pos.getRotMat().toDirectXMatrix();
		XMFLOAT3X3 r;
		DirectX::XMStoreFloat3x3(&r, r4x4);
		return r * inverse_inertia * transpose(r);
	}
	XMMATRIX getXMMatrixRotationQuaternion() {
		return XMMatrixRotationQuaternion(ang_pos.toDirectXQuat());
	}
	XMMATRIX getXMMatrixScaling() const {
		return XMMatrixScaling(size[0], size[1], size[2]);
	}
	XMMATRIX getXMMatrixTranslation() const {
		return XMMatrixTranslation(pos[0], pos[1], pos[2]);
	}
	XMMATRIX getObjToWorldMatrix() {
		const XMMATRIX Rotation = getXMMatrixRotationQuaternion();
		const XMMATRIX Scale = getXMMatrixScaling();
		const XMMATRIX Translation = getXMMatrixTranslation();
		const XMMATRIX ObjToWorld = Scale * Rotation * Translation;
		return ObjToWorld;
	}
	GamePhysics::Mat4 getObjToWorldMat4Matrix() {
		XMMATRIX ObjToWorld = getObjToWorldMatrix();
		return GamePhysics::Mat4(ObjToWorld);
	}

private:
	static XMFLOAT3X3 getCuboidIntertia(Vec3 size) {
		double h = size.y;
		double w = size.x;
		double d = size.z;
		XMFLOAT3X3 res;
		res._11 = 1.0/12*(h*h+d*d);
		res._12 = 0;
		res._13 = 0;
		res._21 = 0;
		res._22 = 1.0/12*(w*w+d*d);
		res._23 = 0;
		res._31 = 0;
		res._32 = 0;
		res._33 = 1.0/12*(w*w+h*h);
		return res;
	}
};

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass); // size in {x, y, z}
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);
	void setAngularVelocityOf(int i, Vec3 ang_vel);
private:
	void integratePosition(float ts);
	void integrateVelocity(float ts);
	void integrateOrientation(float ts);
	void integrateAngularMomentum(float ts);
	void computeAngularVelocity();
	void resolveCollisions();
	void clearStateForNextIteration();

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;
	std::vector<Body> bodies;

	// UI Attributes
	Point2D m_mouse{};
	Point2D m_trackmouse{};
	Point2D m_oldtrackmouse{};
	};
#endif