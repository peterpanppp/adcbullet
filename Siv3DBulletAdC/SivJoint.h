#pragma once

# include <Siv3D.hpp>
# include "BulletCollision/CollisionDispatch/btGhostObject.h"
# include "Block.h"
# include "BlockJoint.h"
# include "JointSupport.hpp"
# include "ConstraintDemo.h"
# include "btBulletDynamicsCommon.h"
# include "LinearMath/btIDebugDraw.h"

# include "SivBlock.h"

class SivJoint
{
public:
	SivJoint(SivBlock& box0, SivBlock& box1)
	{
		Vec3 iniBoxPos0 = box0.getPosSiv3d();
		Vec3 iniBoxPos1 = box1.getPosSiv3d();
		Vec3 center = (iniBoxPos1 + iniBoxPos0) * 0.5;
		Vec3 chainA = center - iniBoxPos0;
		Vec3 chainB = center - iniBoxPos1;
		btVector3 pivotInA(chainA.x, chainA.y, chainA.z);
		btVector3 pivotInB(chainB.x, chainB.y, chainB.z);
		p2p = new btPoint2PointConstraint(*(box0.getRigidBodyPtr()), *(box1.getRigidBodyPtr()), pivotInA, pivotInB);
	}
	SivJoint(SivBlock& box0, SivBlock& box1, const Vec3& pivot0, const Vec3& pivot1)
	{
		btVector3 pivotInA(pivot0.x, pivot0.y, pivot0.z);
		btVector3 pivotInB(pivot1.x, pivot1.y, pivot1.z);
		p2p = new btPoint2PointConstraint(*(box0.getRigidBodyPtr()), *(box1.getRigidBodyPtr()), pivotInA, pivotInB);
	}
	~SivJoint()
	{
		delete p2p;
	}
	btTypedConstraint* getbtPoint2PointConstraint()
	{
		return p2p;
	}
	void setBreakingImpulseThreshold(double force)
	{
		p2p->setBreakingImpulseThreshold(force);
	}
private:
	btTypedConstraint* p2p;
};

