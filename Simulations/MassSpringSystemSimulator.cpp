#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
}

const char* MassSpringSystemSimulator::getTestCasesStr() {
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
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	for (auto& point : m_vPoints)
		point.force = m_externalForce - m_fDamping * point.velocity;
	for (size_t i = 0; i < m_vSprings.size(); i++) {
		Vec3 force = computeElasticForce(m_vSprings[i]);
		m_vPoints[m_vSprings[i].point1].force += force;
		m_vPoints[m_vSprings[i].point2].force -= force;
	}
	integratePositions(timeStep);
	integrateVelocities(timeStep);
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

Vec3 MassSpringSystemSimulator::computeElasticForce(Spring s)
{
	Vec3 p1 = m_vPoints[s.point1].position;
	Vec3 p2 = m_vPoints[s.point2].position;
	float curLength = sqrt(p1.squaredDistanceTo(p2));
	Vec3 dir = (p2 - p1)/curLength;
	return m_fStiffness * (curLength - s.initialLength)*dir;
}

void MassSpringSystemSimulator::integratePositionsEuler(float ts) {
	for (auto& point : m_vPoints) {
		point.position += point.velocity * ts;
	}
}

void MassSpringSystemSimulator::integratePositions(float ts) {
	if (m_iIntegrator == EULER)
		integratePositionsEuler(ts);
	else
		throw "Not implemented";
}

void MassSpringSystemSimulator::integrateVelocitiesEuler(float ts) {
	for (auto& point : m_vPoints) {
		Vec3 accel = point.force / m_fMass;
		point.velocity += accel * ts;
	}
}

void MassSpringSystemSimulator::integrateVelocities(float ts) {
	if (m_iIntegrator == EULER)
		integrateVelocitiesEuler(ts);
	else
		throw "Not implemented";
}