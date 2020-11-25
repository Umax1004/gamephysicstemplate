#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo,Something,TESTCASEUSEDTORUNTEST";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	cout << bodies.size() << endl;
	for (int i = 0; i < bodies.size(); i++)
	{
		XMMATRIX Rotation = XMMatrixRotationQuaternion(bodies[i].ang_pos.toDirectXQuat());
		XMMATRIX Scale = XMMatrixScaling(bodies[i].size[0], bodies[i].size[1], bodies[i].size[2]);
		XMMATRIX Translation = XMMatrixTranslation(bodies[i].pos[0], bodies[i].pos[1], bodies[i].pos[2]);
		XMMATRIX ObjToWorld = Scale * Rotation * Translation;
		DUC->drawRigidBody(ObjToWorld);
		cout << "Position of " << i << " " << bodies[i].pos << endl;
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	bodies.clear();
	switch (testCase)
	{
	case 0:
	{
		addRigidBody({ 0, 0, 0 }, { 3, 1, 1 }, 10);
		setVelocityOf(0, { 1, 0, 0 });
		setOrientationOf(0, Quat{ 3.14 / 4, 3.14 / 4 });
		break;
	}
	case 1:
	{
		addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		addRigidBody(Vec3(0.0f, 0.2f, 0.0f), Vec3(0.4f, 0.2f, 0.2f), 100.0);
		setOrientationOf(1, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.25f));
		setVelocityOf(1, Vec3(0.0f, -0.1f, 0.05f));
		break;
	}
	default:
		break;
	}
	
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	integratePosition();
	integrateVelocity();
	integrateOrientation();
	integrateAngularMomentum();
	computeAngularVelocity();
	resolveCollisions();
	clearStateForNextIteration();
}

void RigidBodySystemSimulator::integratePosition() {
	for (Body& body : bodies)
		body.pos += *m_fTimestep * body.vel;
}

void RigidBodySystemSimulator::integrateVelocity() {
	for (Body& body : bodies)
		body.vel += *m_fTimestep * body.force * body.inverse_mass;
}

void RigidBodySystemSimulator::integrateOrientation() {
	for (Body& body : bodies)
		body.ang_pos += *m_fTimestep / 2 * Quat{ 0, body.ang_vel.x, body.ang_vel.y, body.ang_vel.z } *body.ang_pos;
}

void RigidBodySystemSimulator::integrateAngularMomentum() {
	for (Body& body : bodies)
		body.ang_mom += *m_fTimestep * body.torque;
}

void RigidBodySystemSimulator::computeAngularVelocity() {
	for (Body& body : bodies) {
		auto inverse_inertia = body.getRotatedInverseIntertia();
		body.ang_vel = inverse_inertia * body.ang_mom;
	}
}

void RigidBodySystemSimulator::resolveCollisions() {

}

void RigidBodySystemSimulator::clearStateForNextIteration()
{
	for (Body& body : bodies)
	{
		body.force = 0;
		body.torque = 0;
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies() {
	return bodies.size();
}
Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	return bodies[i].pos;
}
Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	return bodies[i].vel;
}
Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	return bodies[i].ang_vel;
}
void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	bodies[i].force += force;
	bodies[i].torque += cross(loc-bodies[i].pos, force);
}
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {
	bodies.push_back(Body{position, size, (double)mass});
}
void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	bodies[i].ang_pos = orientation;
}
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
	bodies[i].vel = velocity;
}
void RigidBodySystemSimulator::setTimestepVariable(float& timestep)
{
	m_fTimestep = &timestep;
}