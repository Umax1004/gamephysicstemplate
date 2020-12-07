#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "DEMO1,Collision,TESTCASEUSEDTORUNTEST,IAT,Bounce, DEMO4";
}

void TW_CALL RigidBodySystemSimulator::GetGravityCallback(void* value, void* clientData)
{

	static_cast<float*> (value)[0] = static_cast<const Vec3*>(clientData)->x;
	static_cast<float*> (value)[1] = static_cast<const Vec3*>(clientData)->y;
	static_cast<float*> (value)[2] = static_cast<const Vec3*>(clientData)->z;
	//cout << "Get called" << endl;
}

void TW_CALL RigidBodySystemSimulator::SetGravityCallback(const void* value, void* clientData)
{
	static_cast<Vec3*> (clientData)->x = static_cast<const float*> (value)[0];
	static_cast<Vec3*> (clientData)->y = static_cast<const float*> (value)[1];
	static_cast<Vec3*> (clientData)->z = static_cast<const float*> (value)[2];
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;

	TwAddVarRW(DUC->g_pTweakBar, "Bounciness", TW_TYPE_FLOAT, &m_fBounciness, " min=0 max=1 group='Simulation Params' label='Co-efficient of restitution'");
	TwAddVarRW(DUC->g_pTweakBar, "Rotational Friction", TW_TYPE_FLOAT, &m_fRotationalFriction, " min=0 max=1 group='Simulation Params' label='Co-efficient of angular friction'");
	TwAddVarCB(DUC->g_pTweakBar, "Gravity", TW_TYPE_DIR3F, SetGravityCallback, GetGravityCallback, &m_gravity, "group='Simulation Params' label='Gravity'");
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->beginLine();

	/*for (const auto& t : impulses)
	{
		Vec3 position = std::get<1>(t);
		Vec3 direction = std::get<2>(t);
		DUC->drawLine(position.toDirectXVector(), Colors::Red, (position + 0.5 * direction).toDirectXVector(), Colors::Blue);
	}*/

	for (const auto& p : collisions)
	{
		const auto& col = p.first;
		for (int i = 0; i < col.collisionPoints.first; i++) {
			Vec3 collisionPoint = col.collisionPoints.second[i];
			DUC->drawLine(collisionPoint.toDirectXVector(), Colors::Red, (collisionPoint + 0.5 * col.normalWorld).toDirectXVector(), Colors::Blue);
		}
	}

	DUC->endLine();

	//cout << "Number of bodies: " << bodies.size() << endl;
	for (int i = 0; i < bodies.size(); i++)
	{
		if (bodies[i].isMovable)
		{
			XMMATRIX Rotation = XMMatrixRotationQuaternion(bodies[i].ang_pos.toDirectXQuat());
			XMMATRIX Scale = XMMatrixScaling(bodies[i].size[0], bodies[i].size[1], bodies[i].size[2]);
			XMMATRIX Translation = XMMatrixTranslation(bodies[i].pos[0], bodies[i].pos[1], bodies[i].pos[2]);
			XMMATRIX ObjToWorld = Scale * Rotation * Translation;
			DUC->drawRigidBody(ObjToWorld);
			//cout << "Position of " << i << " " << bodies[i].pos << endl;
		}
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_fRotationalFriction = 1;
	bodies.clear();
	switch (testCase)
	{
	case 0:
	{
		addRigidBody({ 0, 0, 0 }, { 1, 0.6, 0.5 }, 2);
		applyForceOnBody(0, { 0.3, 0.5, 0.25 }, { 1,1,0 });
		setVelocityOf(0, { -0.3, -0.5, -0.25 });
		setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5f));
		break;
	}
	case 1:
	{
		addRigidBody(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.2f, 0.2f, 0.2f), 100.0f);
		addRigidBody(Vec3(0.0f, 0.4f, 0.0f), Vec3(0.2f, 0.2f, 0.2f), 500.0f);
		addRigidBody(Vec3(0.0f, -1.5, 0.0f), Vec3(20, 1, 20), INFINITY);
		setOrientationOf(1, Quat(Vec3(0.0f, 1.0f, 1.0f), (float)(M_PI) * 0.25f));
		setVelocityOf(1, Vec3(0.0f, -1, 0.00f));
		setGravity({ 0, -15, 0 });
		m_fRotationalFriction = 0.98;
		break;
	}
	case 2:
	{
		// Nothing. The unit tests use this and expect an empty scene.
		break;
	}
	case 3:
	{
		setGravity({});
		addRigidBody({ 0, 0, 0 }, { 0.5, 0.2, 0.01 }, 10);
		//setAngularVelocityOf(0, { 5, -5, 50 });
		setAngularVelocityOf(0, { 5, 50, 5 });
		//setOrientationOf(0, Quat{ 3.14 / 4, 3.14 / 4 });
		break;
	}
	case 4:
	{
		addRigidBody({ 0, 0.4, 0 }, { 0.2, 0.2, 0.2 }, 100);
		addRigidBody(Vec3(0.0f, -1.5, 0.0f), Vec3(1.5, 1, 1.5), INFINITY);
		//setOrientationOf(0, Quat{ Vec3{1, 1, 1}, 0.001 });
		m_fBounciness = 0.75;
		setGravity({ 0, -3, 0 });
		m_fRotationalFriction = 0.98;
		break;
	}
	case 5:
	{
		addRigidBody({ 0, 0, 0 }, { 0.2, 0.2, 0.2 }, 2);
		addRigidBody({ 0.15, 0.3, 0 }, { 0.2, 0.2, 0.2 }, 2);
		addRigidBody({ 0.25, 0.6, 0 }, { 0.2, 0.2, 0.2 }, 2);
		addRigidBody({ 0.35, 1, 0 }, { 0.2, 0.2, 0.2 }, 2);
		addRigidBody(Vec3(0.0f, -1.5, 0.0f), Vec3(10, 1, 10), INFINITY);
		m_fBounciness = 1;
		setGravity({ 0, -9.8, 0 });
		m_fRotationalFriction = 0.98;
		break;
	}
	default:
		break;
	}
	
}

void RigidBodySystemSimulator::interactiveForcesCalculations()
{
	if (!m_clicked)
	{
		return;
	}
	m_clicked = false;
	const float forceScale = 50;

	for (Body& body : bodies) {
		if (body.isMovable) {
			const Vec3 interactiveForce = { 0 - body.pos.x , 0 - body.pos.y, 0 - body.pos.z };
			const double length = std::sqrt(body.pos.x * body.pos.x + body.pos.y * body.pos.y + body.pos.z * body.pos.z);
			body.force += forceScale * length * interactiveForce / body.inverse_mass;
		}
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	for (Body& body : bodies)
		if (body.isMovable)
			body.force += m_gravity / body.inverse_mass; // Gravity acts on the center of mass, ergo no torque.
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	interactiveForcesCalculations();
	externalForcesCalculations(timeStep);
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
		if(body.isMovable)
			body.pos += ts * body.vel;
}

void RigidBodySystemSimulator::integrateVelocity(float ts) {
	for (Body& body : bodies) {
		if (body.isMovable) {
			body.vel += ts * body.force * body.inverse_mass;
		}
	}
}

void RigidBodySystemSimulator::integrateOrientation(float ts) {
	for (Body& body : bodies) {
		if (body.isMovable) {
			auto ang_velocity = Quat{ body.ang_vel.x, body.ang_vel.y, body.ang_vel.z, 0 };
			body.ang_pos += ts / 2 * ang_velocity * body.ang_pos;
			body.ang_pos = body.ang_pos.unit();
		}
	}
}

void RigidBodySystemSimulator::integrateAngularMomentum(float ts) {
	for (Body& body : bodies)
		if (body.isMovable)
		{
			body.ang_mom += ts * body.torque;
			body.ang_mom *= m_fRotationalFriction;
		}
		else
			body.ang_mom = Vec3();
}

void RigidBodySystemSimulator::computeAngularVelocity() {
	for (Body& body : bodies) {
		auto inverse_inertia = body.getRotatedInverseIntertia();
		body.ang_vel = inverse_inertia * body.ang_mom;
	}
}

static double calculateRelativeVelocity(Body a, Body b, Vec3 point, Vec3 normalWorld) {
	// 1. Calculate velocities at collision point
	const Vec3 thisPosA = point - a.pos;
	const Vec3 thisPosB = point - b.pos;
	const Vec3 thisPointVelocityA = a.vel + cross(a.ang_vel, thisPosA);
	const Vec3 thisPointVelocityB = b.vel + cross(b.ang_vel, thisPosB);

	// 2. Calculate relative velocity
	return dot(normalWorld, thisPointVelocityA - thisPointVelocityB);
}

void RigidBodySystemSimulator::resolveCollisions() {
	for (auto it = collisions.begin(); it != collisions.end();)
	{
		it->second++;
		if (it->second >= 2) {
			it = collisions.erase(it);
			continue;
		}
		it++;
	}
	for (auto it = impulses.begin(); it != impulses.end();)
	{
		std::get<0>(*it)++;
		if (std::get<0>(*it) >= 2) {
			it = impulses.erase(it);
			continue;
		}
		it++;
	}
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
			if (ci.isValid && ci.collisionPoints.first > 0)
			{
				collisions.push_back({ ci , 0});
				// TODO Verify with manual calculation
				const Vec3 centerOfMassVelA = a.vel;
				const Vec3 centerOfMassVelB = b.vel;
				Vec3 collisionPoint;
				//Find the deepest corner that has a colliding contact (relative velocity < 0)
				collisionPoint = {INFINITY, INFINITY, INFINITY};
				double collisionPointDistance = INFINITY;
				Vec3 otherCenter = ci.into ? b.pos : a.pos;
				for (int i = 0; i < ci.collisionPoints.first; i++) {
					Vec3 thisPoint = ci.collisionPoints.second[i];
					if (calculateRelativeVelocity(a, b, thisPoint, ci.normalWorld) < 0) {
						double distToCenter = (thisPoint - otherCenter).norm();
						if (distToCenter < collisionPointDistance) {
							collisionPoint = thisPoint;
							collisionPointDistance = distToCenter;
						}
					}
				}

				if (isfinite(collisionPoint.x)) // == relativeVelocity < 0
				{
					const Vec3 collisionPosA = collisionPoint - a.pos;
					const Vec3 collisionPosB = collisionPoint - b.pos;
					double relativeVelocity = calculateRelativeVelocity(a, b, collisionPoint, ci.normalWorld);

					assert(relativeVelocity < 0);

					// 3. Fill in impulse formula
					const Vec3 normalOfTheCollision = ci.normalWorld;
					const double numerator = -1 * (1 + m_fBounciness) * relativeVelocity;
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

					if (a.isMovable)
						a.vel = newVelocityA;
					if (b.isMovable)
						b.vel = newVelocityB;

					a.ang_mom = newAngularMomentumA;
					b.ang_mom = newAngularMomentumB;
					impulses.push_back({0, collisionPoint, normalOfTheCollision});
				}

				Vec3 deepestCorner;
				//Find the deepest corner (no matter the type of contact)
				deepestCorner = { INFINITY, INFINITY, INFINITY };
				double deepestCornerDistance = INFINITY;
				for (int i = 0; i < ci.collisionPoints.first; i++) {
					Vec3 thisPoint = ci.collisionPoints.second[i];
					double distToCenter = (thisPoint - otherCenter).norm();
					if (distToCenter < deepestCornerDistance) {
						deepestCorner = thisPoint;
						deepestCornerDistance = distToCenter;
					}
				}

				if (a.isMovable)
					a.pos += ci.depth * ci.normalWorld * a.inverse_mass / (a.inverse_mass + b.inverse_mass);
				if (b.isMovable)
					b.pos -= ci.depth * ci.normalWorld * b.inverse_mass / (a.inverse_mass + b.inverse_mass);

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
	m_clicked = true;
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
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, double mass) {
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

void RigidBodySystemSimulator::setGravity(Vec3 force) {
	m_gravity = force;
}