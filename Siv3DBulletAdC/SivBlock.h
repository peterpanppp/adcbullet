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

struct btBoxData
{
	Vec3 halfSize;
	Vec3 center;
	Color color;
};

class btBox
{
public:
	btBox(const Vec3& halfSize = Vec3(1, 1, 1), const Vec3& boxCenter = Vec3::Zero, double weight = 0, Color color = Palette::White)
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
		fallRigidBodyBox->setActivationState(DISABLE_DEACTIVATION);
		data.center = center;
		data.halfSize = size;		
		data.color = color;
	}
	~btBox()
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
	Vec3 getSizeSiv3d() const
	{
		return size;
	}
	btVector3 getSizeBullet() const
	{
		return btVector3(size.x, size.y, size.z);
	}
	btBoxData getData() const
	{
		return data;
	}
	void setColor(ColorF color)
	{
		data.color = color;
	}
private:
	btRigidBody* fallRigidBodyBox;
	btDefaultMotionState* fallMotionStateBox;
	btBoxShape* box;
	Vec3 center;
	Vec3 size;
	double mass;
	btBoxData data;
};

