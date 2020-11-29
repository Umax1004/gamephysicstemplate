#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo,Collision,TESTCASEUSEDTORUNTEST,IAT";
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
	//cout << "Number of bodies: " << bodies.size() << endl;
	for (int i = 0; i < bodies.size(); i++)
	{
		XMMATRIX Rotation = XMMatrixRotationQuaternion(bodies[i].ang_pos.toDirectXQuat());
		XMMATRIX Scale = XMMatrixScaling(bodies[i].size[0], bodies[i].size[1], bodies[i].size[2]);
		XMMATRIX Translation = XMMatrixTranslation(bodies[i].pos[0], bodies[i].pos[1], bodies[i].pos[2]);
		XMMATRIX ObjToWorld = Scale * Rotation * Translation;
		DUC->drawRigidBody(ObjToWorld);
		//cout << "Position of " << i << " " << bodies[i].pos << endl;
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	bodies.clear();
	switch (testCase)
	{
	case 0:
	{
		addRigidBody({ 0, 0, 0 }, { 0.3, 0.1, 0.1 }, 10);
		setVelocityOf(0, { 1, 0, 0 });
		setOrientationOf(0, Quat{ 3.14 / 4, 3.14 / 4 });
		break;
	}
	case 1:
	{
		addRigidBody(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.2f, 0.2f, 0.2f), 100.0f);
		addRigidBody(Vec3(0.0f, 0.4f, 0.0f), Vec3(0.2f, 0.2f, 0.2f), 100.0f);
		setOrientationOf(1, Quat(Vec3(0.0f, 1.0f, 1.0f), (float)(M_PI) * 0.25f));
		setVelocityOf(1, Vec3(0.0f, -0.2f, 0.00f));
		break;
	}
	case 2:
	{
		// Nothing. The unit tests use this and expect an empty scene.
		break;
	}
	case 3:
	{
		addRigidBody({ 0, 0, 0 }, { 0.5, 0.2, 0.01 }, 10);
		//setAngularVelocityOf(0, { 5, -5, 50 });
		setAngularVelocityOf(0, { 5, 50, 5 });
		//setOrientationOf(0, Quat{ 3.14 / 4, 3.14 / 4 });
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
	integratePosition(timeStep);
	integrateVelocity(timeStep);
	integrateOrientation(timeStep);
	integrateAngularMomentum(timeStep);
	computeAngularVelocity();
	resolveCollisions();
	clearStateForNextIteration();
}

void RigidBodySystemSimulator::integratePosition(float ts) {
	for (Body& body : bodies)
		body.pos += ts * body.vel;
}

void RigidBodySystemSimulator::integrateVelocity(float ts) {
	for (Body& body : bodies)
		body.vel += ts * body.force * body.inverse_mass;
}

void RigidBodySystemSimulator::integrateOrientation(float ts) {
	for (Body& body : bodies) {
		body.ang_pos += ts / 2 * Quat{ body.ang_vel.x, body.ang_vel.y, body.ang_vel.z, 0 } * body.ang_pos;
		body.ang_pos = body.ang_pos.unit();
	}
}

void RigidBodySystemSimulator::integrateAngularMomentum(float ts) {
	for (Body& body : bodies)
		body.ang_mom += ts * body.torque;
}

void RigidBodySystemSimulator::computeAngularVelocity() {
	for (Body& body : bodies) {
		auto inverse_inertia = body.getRotatedInverseIntertia();
		body.ang_vel = inverse_inertia * body.ang_mom;
	}
}

void RigidBodySystemSimulator::resolveCollisions() {
	if (bodies.size() < 2)
	{
		return;
	}
	for (int i = 0; i < bodies.size() - 1; ++i)
	{
		Body& a = bodies[i];
		Mat4 MatrixA = a.getObjToWorldMat4Matrix();

		for (auto j = i + 1; j < bodies.size(); ++j)
		{
			Body& b = bodies[j];
			Mat4 MatrixB = b.getObjToWorldMat4Matrix();

			CollisionInfo ci = checkCollisionSAT(MatrixA, MatrixB);
			if (ci.isValid)
			{
				// TODO Verify with manual calculation
				// 1. Calculate velocities at collision point
				const Vec3 centerOfMassVelA = a.vel;
				const Vec3 centerOfMassVelB = b.vel;
				const Vec3 collisionPoint = ci.collisionPointWorld;
				const Vec3 collisionPosA = collisionPoint - a.pos;
				const Vec3 collisionPosB = collisionPoint - b.pos;
				const Vec3 collisionPointVelocityA = centerOfMassVelA + cross(a.ang_vel, collisionPosA);
				const Vec3 collisionPointVelocityB = centerOfMassVelB + cross(b.ang_vel, collisionPosB);

				// 2. Calculate relative velocity
				const Vec3 relativeVelocity = collisionPointVelocityA - collisionPointVelocityB;


				if (relativeVelocity.x < 0 || relativeVelocity.y < 0 || relativeVelocity.x < 0)
				{

				}
				else
				{
					// 3. Fill in impulse formula
					const Vec3 normalOfTheCollision = ci.normalWorld;
					const double c = 1.0f; // TODO consider to convert it to a parameter
					const double numerator = -1 * (1 + c) * dot(relativeVelocity, normalOfTheCollision);
					const auto inverseInertiaA = a.inverse_inertia;
					const auto inverseInertiaB = b.inverse_inertia;
					const auto denominatorPartA = cross(inverseInertiaA * cross(collisionPosA, normalOfTheCollision), collisionPosA);
					const auto denominatorPartB = cross(inverseInertiaB * cross(collisionPosB, normalOfTheCollision), collisionPosB);
					const double denominator = a.inverse_mass + b.inverse_mass + dot(denominatorPartA + denominatorPartB, normalOfTheCollision);
					const double impulse = numerator / denominator;
					// 4. Apply impulse
					const Vec3 newVelocityA = centerOfMassVelA + impulse * normalOfTheCollision * a.inverse_mass;
					const Vec3 newVelocityB = centerOfMassVelB - impulse * normalOfTheCollision * b.inverse_mass;
					const Vec3 newAngularMomentumA = a.ang_mom + cross(collisionPosA, impulse * normalOfTheCollision);
					const Vec3 newAngularMomentumB = b.ang_mom - cross(collisionPosB, impulse * normalOfTheCollision);

					a.vel = newVelocityA;
					b.vel = newVelocityB;
					a.ang_mom = newAngularMomentumA;
					b.ang_mom = newAngularMomentumB;
				}
				
				
				//cout << relativeVelocity << endl;
				
			}
		}
	}
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
void RigidBodySystemSimulator::setAngularVelocityOf(int i, Vec3 ang_vel) {
	// We cannot just set ang_vel here, because it is used just as a cache and will be overwritten
	auto inertia = inverse(bodies[i].getRotatedInverseIntertia());
	bodies[i].ang_mom = inertia * ang_vel;
	bodies[i].ang_vel = ang_vel;
}