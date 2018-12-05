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
	SivBlock(const Vec3& halfSize = Vec3(1, 1, 1), const Vec3& boxCenter = Vec3::Zero, double weight = 0)
	{
		size = halfSize;
		box = new btBoxShape(btVector3(size.x, size.y, size.z));
		Vec3 iniBoxPos(boxCenter);
		center = iniBoxPos;
		fallMotionStateBox =
			new btDefaultMotionState(
				btTransform(btQuaternion(0, 0, 0, 1), bt2s3d(iniBoxPos)));
		mass = weight;
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
	Vec3 getPosSiv3d() const
	{
		return center;
	}
	btVector3 getPosBullet() const
	{
		return btVector3(center.x, center.y, center.z);
	}
private:
	btRigidBody* fallRigidBodyBox;
	btDefaultMotionState* fallMotionStateBox;
	btBoxShape* box;
	Vec3 center;
	Vec3 size;
	double mass;
};

