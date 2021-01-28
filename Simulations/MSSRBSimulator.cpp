#include "MSSRBSimulator.h"

MSSRBSimulator::MSSRBSimulator()
{
	m_iTestCase = 0;
}

const char* MSSRBSimulator::getTestCasesStr() {
	return "PENDULUM,DEMO,COMPLEX,CUBE,BOUNCE";
}

const char* MSSRBSimulator::getIntegratorsStr() {
	return "EULER,LEAPFROG,MIDPOINT";
}

void MSSRBSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	rb.initUI(DUC);
	mss.initUI(DUC);

	/*TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, " min=0 group='Simulation Params' label='Spring stiffness co-efficient'");
	TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, " min=0 group='Simulation Params' label='Mass of Points'");
	TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, " min=0 group='Simulation Params' label='Damping of Spring'");
	TwAddVarCB(DUC->g_pTweakBar, "Gravity", TW_TYPE_DIR3F, SetGravityCallback, GetGravityCallback , &m_externalForce, "group='Simulation Params' label='Gravity'");*/
}

void MSSRBSimulator::reset()
{
	rb.reset();
	mss.reset();
}

void MSSRBSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	mss.drawSprings(pd3dImmediateContext);
	rb.drawFrame(pd3dImmediateContext);
}

void MSSRBSimulator::notifyCaseChanged(int testCase)
{
	rb.m_fRotationalFriction = 1;
	rb.setGravity({});
	rb.m_fFriction = 1;
	rb.bodies.clear();

	mss.m_vPoints.clear();
	mss.m_vSprings.clear();

	m_iTestCase = testCase;

	if (m_iTestCase == 0) {
		setMass(1.0f);
		setDampingFactor(1);
		setStiffness(30.0f);
		setGravity({ 0, -3, 0 });
		int p0 = addRigidBody(Vec3(-0.5, 0.5f, 0), Vec3(0.2, 0.2f, 0.2f));
		int p1 = addRigidBody(Vec3(0.0, 0.5f, 0), Vec3(0.2, 0.2f, 0.2));
		addSpring(p0, p1, 1);
	}
	else if (m_iTestCase == 1) { // Basic Mass Spring
		setMass(10.0f);
		setDampingFactor(0);
		setStiffness(40.0f);
		setGravity(Vec3());
		int p0 = addRigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1));
		int p1 = addRigidBody(Vec3(0, 2, 0), Vec3(0.1, 0.1, 0.1));
		addSpring(p0, p1, 1);
		isFirst = true;
	}
	else if (m_iTestCase == 2) { //Complex

		setMass(0.5f);
		setDampingFactor(0.5);
		setStiffness(500.0f);

		const int massPointsAmount = 10;
		const float wholeSpringLength = 0.6f;
		float x = 0.f, y = 0.5f;
		const float step = wholeSpringLength / static_cast<float>(massPointsAmount);
		int previousPointId = addRigidBody(Vec3(x, y, 0), Vec3(0.01, 0.01, 0.01));
		for (int i = 0; i < massPointsAmount; ++i)
		{
			x -= step;
			y -= step / 2;
			const int currentPointId = addRigidBody(Vec3(x, y, 0), Vec3(0.01, 0.01, 0.01));
			addSpring(previousPointId, currentPointId, wholeSpringLength / massPointsAmount);
			previousPointId = currentPointId;
		}

	}
	else if (m_iTestCase == 3) { // Cube
		setMass(1.0f);
		setDampingFactor(1);
		setStiffness(300.0f);
		setGravity(Vec3{ 0, -9.8, 0 });
		for (int y = 0; y < 3; y++)
		{
			for (int x = 1; x >= -1; x--)
			{
				for (int z = -1; z <= 1; z++)
				{
					int temp = addRigidBody(Vec3(x, y, z), {0.2, 0.2, 0.2});
				}
			}
		}

		for (int y = 0; y <= 2; y++)
		{
			for (int x = 0; x <= 2; x++)
			{
				for (int z = 0; z <= 2; z++)
				{
					if (y != 0)
					{
						addSpring((y * 9) + (x * 3) + z, ((y - 1) * 9) + (x * 3) + z, 1);
					}
					if (x != 2)
					{
						addSpring((y * 9) + (x * 3) + z, (y * 9) + ((x+1) * 3) + z, 1);
					}
					if (z != 2)
					{
						addSpring((y * 9) + (x * 3) + z, (y * 9) + (x * 3) + (z+1), 1);
					}
					//addSpring(p0, p1, );
				}
			}
		}
		
	}
	else if (m_iTestCase == 4) {
		setMass(0.5f);
		setDampingFactor(0.5);
		setGravity(Vec3{ 0, -9.8, 0 });
		setGravity(Vec3{ 0, -9.8, 0 });
		addRigidBody({ 0, 0.5, 0 }, {0.2, 0.2, 0.2});
	}
	else
		throw "Not implemented";
}

void MSSRBSimulator::externalForcesCalculations(float timeElapsed)
{
	rb.externalForcesCalculations(timeElapsed);
}

void MSSRBSimulator::simulateTimestep(float ts)
{
	mss.computeForces();
	// Transfer the forces computed by the MSS over to RB
	assert(rb.bodies.size() == mss.m_vPoints.size());
	for (int i = 0; i < rb.bodies.size(); i++) {
		rb.bodies[i].force += mss.m_vPoints[i].force; // We add the forces here, because calculateExternalForces already adds some forces (gravity, for example)
	}
	rb.simulateTimestep(ts);
	// Transfer the positions and velocities computed by RB over to the MSS
	assert(rb.bodies.size() == mss.m_vPoints.size());
	for (int i = 0; i < rb.bodies.size(); i++) {
		mss.m_vPoints[i].position = rb.bodies[i].pos;
		mss.m_vPoints[i].velocity = rb.bodies[i].vel;
	}
}

void MSSRBSimulator::onClick(int x, int y)
{
	rb.onClick(x, y);
}

void MSSRBSimulator::onMouse(int x, int y)
{
	rb.onMouse(x, y);
}

void MSSRBSimulator::setGravity(Vec3 g) {
	rb.setGravity(g);
}
void MSSRBSimulator::setMass(float mass)
{
	mss.setMass(mass);
}
void MSSRBSimulator::setStiffness(float stiffness)
{
	mss.setStiffness(stiffness);
}
void MSSRBSimulator::setDampingFactor(float damping)
{
	mss.setDampingFactor(damping);
}
int MSSRBSimulator::addRigidBody(Vec3 position, Vec3 size)
{
	int i1 = mss.addMassPoint(position, Vec3{0,0,0}, false);
	int i2 = rb.addRigidBody(position, size, mss.m_fMass);
	assert(i1 == i2);
	return i1;
}
void MSSRBSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	mss.addSpring(masspoint1, masspoint2, initialLength);
}

void TW_CALL MSSRBSimulator::GetGravityCallback(void* value, void* clientData)
{
	
	static_cast<float*> (value)[0] = static_cast<const Vec3*>(clientData)->x;
	static_cast<float*> (value)[1] = static_cast<const Vec3*>(clientData)->y;
	static_cast<float*> (value)[2] = static_cast<const Vec3*>(clientData)->z;
	//cout << "Get called" << endl;
}

void TW_CALL MSSRBSimulator::SetGravityCallback(const void* value, void* clientData)
{
	static_cast<Vec3*> (clientData)->x = static_cast<const float*> (value)[0];
	static_cast<Vec3*> (clientData)->y = static_cast<const float*> (value)[1];
	static_cast<Vec3*> (clientData)->z = static_cast<const float*> (value)[2];
}
