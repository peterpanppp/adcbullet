# pragma once
# include <Siv3D.hpp>
# include "btBulletDynamicsCommon.h"
# include "BulletCollision/CollisionDispatch/btGhostObject.h"
# include "Bullet.h"

namespace
{
btVector3 s3d2bt(const Vec3& vec)
{
	return btVector3(vec.x, vec.y, vec.z);
}

Vec3 bt2s3d(const btVector3& vec)
{
	return Vec3(vec.x(), vec.y(), vec.z());
}
}

class Block
{
public:
	Block();
	Block(const Vec3& size, const Vec3& pos, double mass):
		m_btSize(s3d2bt(size)),
		m_btPos(s3d2bt(pos)),
		m_btMass(mass)
	{
		m_boxPtr = std::make_shared<btBoxShape>(m_btSize);
		m_fallMotionStateBoxPtr =
			new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), m_btPos));
		btVector3 fallInertia(0, 0, 0);
		m_boxPtr->calculateLocalInertia(m_btMass, fallInertia);
		btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCIBox
		(m_btMass, m_fallMotionStateBoxPtr, m_boxPtr.get(), fallInertia);
		// çÑëÃÇÃçÏê¨
		m_fallRigidBodyBoxPtr = std::make_shared<btRigidBody>(fallRigidBodyCIBox);

	}
	~Block();
	btRigidBody* getBoxRigidBodyPtr() const { return m_fallRigidBodyBoxPtr.get(); }
	btTransform getTransform() const 
	{
		btTransform trans;
		m_fallRigidBodyBoxPtr->getMotionState()->getWorldTransform(trans);
		return trans;
	}
	Vec3 getPos() const
	{
		auto trans = getTransform();
		auto origin = trans.getOrigin();
		Vec3 pos(origin.getX(), origin.getY(), origin.getZ());
		return pos;
	}
	Quaternion getQuaternion() const
	{
		auto trans = getTransform();
		auto rot = trans.getRotation();
		//Quaternion q(-rot.getX(), rot.getY(), rot.getZ(), -rot.getW());
		Quaternion q(rot.getX(), rot.getY(), rot.getZ(), rot.getW());
		return q;
	}
	void draw() const
	{
		Box(Vec3::Zero, bt2s3d(m_btSize * 2)).asMesh().rotated(getQuaternion()).translated(getPos()).draw();
	}
	void draw(const Color& color) const
	{
		Box(Vec3::Zero, bt2s3d(m_btSize * 2)).asMesh().rotated(getQuaternion()).translated(getPos()).draw(color);
	}
	void drawShadow(const Color& color) const
	{
		Box(Vec3::Zero, bt2s3d(m_btSize * 2)).asMesh().rotated(getQuaternion()).translated(getPos()).drawShadow().draw(color);
	}
private:
	btVector3 m_btSize;
	btVector3 m_btPos;
	btScalar m_btMass;
	std::shared_ptr<btBoxShape> m_boxPtr;
	std::shared_ptr<btRigidBody> m_fallRigidBodyBoxPtr;
	btDefaultMotionState* m_fallMotionStateBoxPtr;
};

