#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
}

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "PENDULUM,DEMO,COMPLEX,CUBE,BOUNCE";
}

const char* MassSpringSystemSimulator::getIntegratorsStr() {
	return "EULER,LEAPFROG,MIDPOINT";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;

	TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, " min=0 group='Simulation Params' label='Spring stiffness co-efficient'");
	TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, " min=0 group='Simulation Params' label='Mass of Points'");
	TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, " min=0 group='Simulation Params' label='Damping of Spring'");
	TwAddVarCB(DUC->g_pTweakBar, "Gravity", TW_TYPE_DIR3F, SetGravityCallback, GetGravityCallback , &m_externalForce, "group='Simulation Params' label='Gravity'");
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	PrimitiveBatch<VertexPositionColor> g_pPrimitiveBatchPositionColor = PrimitiveBatch<VertexPositionColor>(pd3dImmediateContext);

	g_pPrimitiveBatchPositionColor.Begin();
	for (int i = 0; i < m_vSprings.size(); i++)
	{
		//cout << "Line" << m_vPoints[m_vSprings[i].point1].position << "and" << m_vPoints[m_vSprings[i].point2].position << endl;
		//DUC->drawLine(m_vPoints[m_vSprings[i].point1].position, Vec3(1), m_vPoints[m_vSprings[i].point2].position, Vec3(1));
		//DUC->endLine();

		VertexPositionColor v1(m_vPoints[m_vSprings[i].point1].position.toDirectXVector(), Colors::Yellow);
		VertexPositionColor v2(m_vPoints[m_vSprings[i].point2].position.toDirectXVector(), Colors::Yellow);

		g_pPrimitiveBatchPositionColor.DrawLine(v1, v2);

	}
	g_pPrimitiveBatchPositionColor.End();

	for (int i = 0; i < m_vPoints.size(); i++)
	{
		DUC->drawSphere(m_vPoints[i].position, m_vPoints[i].size);
	}

}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_vPoints.clear();
	m_vSprings.clear();
	*m_fTimestep = 0.001f;
	m_iTestCase = testCase;

	if (m_iTestCase == 0) { // Pendulum
		setMass(1.0f);
		setDampingFactor(1);
		setStiffness(30.0f);
		applyExternalForce(Vec3{ 0, -3, 0 });
		int p0 = addMassPoint(Vec3(-0.5, 0.5f, 0), Vec3(0.0, 0.0f, 0), false);
		int p1 = addMassPoint(Vec3(0.0, 0.5f, 0), Vec3(0.0, 0.0f, 0), true);
		addSpring(p0, p1, 0.5);
	}
	else if (m_iTestCase == 1) { // Basic Mass Spring
		setMass(10.0f);
		setDampingFactor(0);
		setStiffness(40.0f);
		applyExternalForce(Vec3());
		int p0 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		int p1 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(p0, p1, 1);
		*m_fTimestep = 0.005f;
		isFirst = true;
		setIntegrator(EULER);
	}
	else if (m_iTestCase == 2) { //Complex

		setMass(0.5f);
		setDampingFactor(0.5);
		setStiffness(500.0f);
		applyExternalForce(Vec3{ 0, -3, 0 });

		const int massPointsAmount = 10;
		const float wholeSpringLength = 0.6f;
		float x = 0.f, y = 0.5f;
		const float step = wholeSpringLength / static_cast<float>(massPointsAmount);
		int previousPointId = addMassPoint(Vec3(x, y, 0), Vec3(), Vec3(0.01), true);
		for (int i = 0; i < massPointsAmount; ++i)
		{
			x -= step;
			y -= step / 2;
			const int currentPointId = addMassPoint(Vec3(x, y, 0), Vec3(), Vec3(0.01), false);
			addSpring(previousPointId, currentPointId, wholeSpringLength / massPointsAmount);
			previousPointId = currentPointId;
		}

	}
	else if (m_iTestCase == 3) { // Cube
		setMass(1.0f);
		setDampingFactor(1);
		setStiffness(300.0f);
		*m_fTimestep = 0.005f;
		applyExternalForce(Vec3{ 0, -9.8, 0 });
		for (int y = 0; y < 3; y++)
		{
			for (int x = 1; x >= -1; x--)
			{
				for (int z = -1; z <= 1; z++)
				{
					int temp = addMassPoint(Vec3(x,y,z), Vec3(), false);
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
		applyExternalForce(Vec3{ 0, -9.8, 0 });
		addMassPoint({ 0, 0.5, 0 }, {}, false);
	}
	else
		throw "Not implemented";
}

/*
* Confusing naming: externalForcesCalculations calculates mouse force
* m_externalForce is the gravity, i.e., the non-interactive component of the "external force"
*/
void MassSpringSystemSimulator::externalForcesCalculations(float /*timeElapsed*/)
{
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// TODO find a proper scale! Rather Turn it into a parameter
		float inputScale = 0.01;
		inputWorld = inputWorld * inputScale;
		m_mouseForce = inputWorld;
	}
	else {
		m_mouseForce = Vec3{};
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	computeForces();
	integrate(timeStep);
	resolveCollisions();
}

void MassSpringSystemSimulator::resolveCollisions() {
	const double GROUND_BOUNCINESS = 0.75;
	for (Point& point : m_vPoints) {
		if (point.position.y < m_fFloor && point.velocity.y < 0)
			point.velocity.y *= -1 * GROUND_BOUNCINESS;
		point.position.y = std::max(point.position.y, (Real)m_fFloor);
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}
void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}
void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed)
{
	return addMassPoint(position, velocity, Vec3(0.1), isFixed);
}
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, Vec3 size, bool isFixed)
{
	m_vPoints.push_back(Point{ position, velocity, {} , size, isFixed });
	return m_vPoints.size()-1;
}
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	assert(masspoint1 >= 0 && masspoint1 < m_vPoints.size());
	assert(masspoint2 >= 0 && masspoint2 < m_vPoints.size());
	m_vSprings.push_back(Spring{ masspoint1, masspoint2, initialLength });
}
int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_vPoints.size();
}
int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_vSprings.size();
}
Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	assert(index >= 0 && index < m_vPoints.size());
	return m_vPoints[index].position;
}
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	assert(index >= 0 && index < m_vPoints.size());
	return m_vPoints[index].velocity;
}
void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce = force;
}

void TW_CALL MassSpringSystemSimulator::GetGravityCallback(void* value, void* clientData)
{
	
	static_cast<float*> (value)[0] = static_cast<const Vec3*>(clientData)->x;
	static_cast<float*> (value)[1] = static_cast<const Vec3*>(clientData)->y;
	static_cast<float*> (value)[2] = static_cast<const Vec3*>(clientData)->z;
	//cout << "Get called" << endl;
}

void TW_CALL MassSpringSystemSimulator::SetGravityCallback(const void* value, void* clientData)
{
	static_cast<Vec3*> (clientData)->x = static_cast<const float*> (value)[0];
	static_cast<Vec3*> (clientData)->y = static_cast<const float*> (value)[1];
	static_cast<Vec3*> (clientData)->z = static_cast<const float*> (value)[2];
}

void MassSpringSystemSimulator::setTimestepVariable(float& timestep)
{
	m_fTimestep = &timestep;
}

void MassSpringSystemSimulator::computeForces() {
	for (auto& point : m_vPoints) {
		point.force = m_mouseForce + m_externalForce - m_fDamping * point.velocity;
	}
	for (size_t i = 0; i < m_vSprings.size(); i++) {
		Vec3 force = computeElasticForce(m_vSprings[i]);
		m_vPoints[m_vSprings[i].point1].force += force;
		m_vPoints[m_vSprings[i].point2].force -= force;
	}
}

Vec3 MassSpringSystemSimulator::computeElasticForce(Spring s)
{
	Vec3 p1 = m_vPoints[s.point1].position;
	Vec3 p2 = m_vPoints[s.point2].position;
	float curLength = sqrt(p1.squaredDistanceTo(p2));
	Vec3 dir = (p2 - p1)/curLength;
	return m_fStiffness * (curLength - s.initialLength)*dir;
}

void MassSpringSystemSimulator::integrate(float ts) {
	if (m_iIntegrator == EULER) {
		integratePositionsEuler(ts);
		integrateVelocitiesEuler(ts);
		if (isFirst && m_iTestCase == 1)
		{
			cout << "Result of Euler's Method" << endl;
			for (int i = 0; i < m_vPoints.size(); i++)
			{
				cout << "Point " << i << " Position: " << m_vPoints[i].position << " Velocity: " << m_vPoints[i].velocity << endl;
			}
			isFirst = false;
		}
	}
	else if (m_iIntegrator == MIDPOINT)
	{
		integrateMidpoint(ts);
		if (isFirst && m_iTestCase == 1)
		{
			cout << "Result of Midpoint's Method" << endl;
			for (int i = 0; i < m_vPoints.size(); i++)
			{
				cout << "Point " << i << " Position: " << m_vPoints[i].position << " Velocity: " << m_vPoints[i].velocity << endl;
			}
			isFirst = false;
		}
	}
	else if (m_iIntegrator == LEAPFROG) {
		integrateVelocitiesEuler(ts);
		integratePositionsEuler(ts);
	}
	else
		throw "Not implemented";
}

void MassSpringSystemSimulator::integratePositionsEuler(float ts) {
	for (auto& point : m_vPoints) {
		if (!point.isFixed)
		{
			point.position += point.velocity * ts;
		}
	}
}

void MassSpringSystemSimulator::integrateMidpoint(float ts) {
	std::vector<Vec3> initialPositions(m_vPoints.size());
	std::vector<Vec3> initialVelocities(m_vPoints.size());
	for (size_t i = 0; i < m_vPoints.size(); i++) {
		initialPositions[i] = m_vPoints[i].position;
		initialVelocities[i] = m_vPoints[i].velocity;
	}
	integratePositionsEuler(ts/2);
	integrateVelocitiesEuler(ts/2);
	computeForces();
	for (size_t i = 0; i < m_vPoints.size(); i++)
		m_vPoints[i].position = initialPositions[i];
	integratePositionsEuler(ts);
	for (size_t i = 0; i < m_vPoints.size(); i++)
		m_vPoints[i].velocity = initialVelocities[i];
	integrateVelocitiesEuler(ts);
}

void MassSpringSystemSimulator::integrateVelocitiesEuler(float ts) {
	for (auto& point : m_vPoints) {
		Vec3 accel = point.force / m_fMass;
		point.velocity += accel * ts;
	}
}