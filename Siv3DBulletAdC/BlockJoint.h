# pragma once
# include <Siv3D.hpp>
# include <btBulletDynamicsCommon.h>
# include "BulletCollision/CollisionDispatch/btGhostObject.h"
# include "Bullet.h"

namespace
{
btVector3 s3d2bt2(const Vec3& vec)
{
	return btVector3(vec.x, vec.y, vec.z);
}

Vec3 bt2s3d2(const btVector3& vec)
{
	return Vec3(vec.x(), vec.y(), vec.z());
}
}

class BlockJoint
{
public:
	BlockJoint();
	BlockJoint(const Vec3& posA, const Vec3& posB, double breakForce, 
		btRigidBody& bodyA, btRigidBody& bodyB) :
		m_posA(s3d2bt2(posA)),
		m_posB(s3d2bt2(posB)),
		m_breakForce(breakForce)
	{
		m_jointPoint2Point = 
			new btPoint2PointConstraint(bodyA, bodyB, m_posA, m_posB);
		m_jointPoint2Point->setBreakingImpulseThreshold(m_breakForce);
	}
	~BlockJoint();
	btTypedConstraint* getJoint() const { return m_jointPoint2Point; }
private:
	btTypedConstraint* m_jointPoint2Point;
	btVector3 m_posA;
	btVector3 m_posB;
	btScalar m_breakForce;
};

