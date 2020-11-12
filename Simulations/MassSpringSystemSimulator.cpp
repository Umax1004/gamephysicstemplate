#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
}

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "PENDULUM";
}

const char* MassSpringSystemSimulator::getIntegratorsStr() {
	return "EULER,LEAPFROG,MIDPOINT";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for (int i = 0; i < m_vSprings.size(); i++)
	{
		//cout << "Line" << m_vPoints[m_vSprings[i].point1].position << "and" << m_vPoints[m_vSprings[i].point2].position << endl;
		//DUC->drawLine(m_vPoints[m_vSprings[i].point1].position, Vec3(1), m_vPoints[m_vSprings[i].point2].position, Vec3(1));
		//DUC->endLine();
		PrimitiveBatch<VertexPositionColor> g_pPrimitiveBatchPositionColor = PrimitiveBatch<VertexPositionColor>(pd3dImmediateContext);

		g_pPrimitiveBatchPositionColor.Begin();

		VertexPositionColor v1(m_vPoints[m_vSprings[i].point1].position.toDirectXVector(), Colors::Yellow);
		VertexPositionColor v2(m_vPoints[m_vSprings[i].point2].position.toDirectXVector(), Colors::Yellow);

		g_pPrimitiveBatchPositionColor.DrawLine(v1, v2);

		g_pPrimitiveBatchPositionColor.End();
	}

	for (int i = 0; i < m_vPoints.size(); i++)
	{
		DUC->drawSphere(m_vPoints[i].position, Vec3(0.1));
	}

	
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_vPoints.clear();
	m_vSprings.clear();

	if (testCase == 0) { // Pendulum
		setMass(1.0f);
		setDampingFactor(1);
		setStiffness(30.0f);
		applyExternalForce(Vec3{ 0, -3, 0 });
		int p0 = addMassPoint(Vec3(-0.5, 0.5f, 0), Vec3(0.0, 0.0f, 0), false);
		int p1 = addMassPoint(Vec3(0.0, 0.5f, 0), Vec3(0.0, 0.0f, 0), true);
		addSpring(p0, p1, 0.5);
	}
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
		// find a proper scale!
		float inputScale = 0.1;
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
	m_vPoints.push_back(Point{ position, velocity, {} , isFixed });
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

void MassSpringSystemSimulator::computeForces() {
	for (auto& point : m_vPoints)
		point.force = m_mouseForce + m_externalForce - m_fDamping * point.velocity;
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
	}
	else if (m_iIntegrator == MIDPOINT)
		integrateMidpoint(ts);
	else
		throw "Not implemented";
}

void MassSpringSystemSimulator::integratePositionsEuler(float ts) {
	for (auto& point : m_vPoints) {
		if (!point.isFixed)
		{	
			point.position += point.velocity * ts;
			point.position.y = std::max(point.position.y, -0.75); // TODO: More advanced collision detection. See the description PDF.
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