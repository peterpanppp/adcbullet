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


class SivbtDynamicWorld
{
public:
	SivbtDynamicWorld();
	~SivbtDynamicWorld();
	void update()
	{
		s3d::Graphics3D::FreeCamera();
		dynamicsWorld->stepSimulation(1 / 60.f, 10);
	}
	void draw() const
	{
		{
			s3d::Line3D(s3d::Vec3(1000, 0, 0), s3d::Vec3(-1000, 0, 0)).drawForward(s3d::Palette::Red);
			s3d::Line3D(s3d::Vec3(0, 1000, 0), s3d::Vec3(0, -1000, 0)).drawForward(s3d::Palette::Blue);
			s3d::Line3D(s3d::Vec3(0, 0, 1000), s3d::Vec3(0, 0, -1000)).drawForward(s3d::Palette::Green);
		}
	}
	void addRigidBody(SivBlock& block);
private:
	btDiscreteDynamicsWorld* dynamicsWorld;
	btSequentialImpulseConstraintSolver* solver;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btAxisSweep3* broadphase;
	btStaticPlaneShape* groundShape;
	btDefaultMotionState* groundMotionState;
	btRigidBody* groundRigidBody;
	btCollisionDispatcher* dispatcher;
};

