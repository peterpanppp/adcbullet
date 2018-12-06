#pragma once

# include <Siv3D.hpp>
# include "BulletCollision/CollisionDispatch/btGhostObject.h"
# include "Block.h"
# include "BlockJoint.h"
# include "JointSupport.hpp"
# include "ConstraintDemo.h"
# include "btBulletDynamicsCommon.h"
# include "LinearMath/btIDebugDraw.h"

class SivSphere
{
public:
	SivSphere(const Vec3& center = Vec3::Zero, double ballRadius = 1, double mass = 1) :
		radius(ballRadius)
	{
		pColShape = new btSphereShape(btScalar(radius));
		fallMotionStateBall =
			new btDefaultMotionState(
				btTransform(btQuaternion(0, 0, 0, 1), btVector3(center.x, center.y, center.z)));
		btVector3 fallInertia(0, 0, 0);
		pColShape->calculateLocalInertia(mass, fallInertia);
		btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCIDynamicBall
		(mass, fallMotionStateBall, pColShape, fallInertia);
		fallRigidBodyBall = new btRigidBody(fallRigidBodyCIDynamicBall);
	}
	~SivSphere()
	{
		delete pColShape;
	}
	double getRadius() const
	{
		return radius;
	}
	btRigidBody* getRigidBodyPtr()
	{
		return fallRigidBodyBall;
	}
private:
	btSphereShape* pColShape;
	btDefaultMotionState* fallMotionStateBall;
	btRigidBody* fallRigidBodyBall;
	double radius;
};

