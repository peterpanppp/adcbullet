#pragma once
# include <Siv3D.hpp>
# include "BulletCollision/CollisionDispatch/btGhostObject.h"
# include "Block.h"
# include "BlockJoint.h"
# include "JointSupport.hpp"
# include "ConstraintDemo.h"
# include "btBulletDynamicsCommon.h"
# include "LinearMath/btIDebugDraw.h"

namespace
{
btVector3 bt2s3d(const Vec3& vec)
{
	return btVector3(vec.x, vec.y, vec.z);
}

}

class SivBlock
{
public:
	SivBlock()
	{
		box = new btBoxShape(btVector3(1, 1, 1));
		Vec3 iniBoxPos(0, 2, 0);
		fallMotionStateBox =
			new btDefaultMotionState(
				btTransform(btQuaternion(0, 0, 0, 1), bt2s3d(iniBoxPos)));
		btScalar mass = 1;
		btVector3 fallInertia(0, 0, 0);
		box->calculateLocalInertia(mass, fallInertia);
		btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCIBox(mass, fallMotionStateBox, box, fallInertia);
		fallRigidBodyBox = new btRigidBody(fallRigidBodyCIBox);
	}
	~SivBlock()
	{
		delete box;
	}
	btRigidBody* getRigidBodyPtr()
	{
		return fallRigidBodyBox;
	}
private:
	btRigidBody* fallRigidBodyBox;
	btDefaultMotionState* fallMotionStateBox;
	btBoxShape* box;
};

