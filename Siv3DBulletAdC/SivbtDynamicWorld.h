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
		for (auto box : btRigidBodies)
		{
			btTransform trans;
			box->getMotionState()->getWorldTransform(trans);
			auto rot = trans.getRotation();
			Quaternion q(rot.getX(), rot.getY(), rot.getZ(), rot.getW());
			auto pos = trans.getOrigin();
			Vec3 point(pos.getX(), pos.getY(), pos.getZ());
			Box(Vec3::Zero, bt2s3d(btVector3(1, 1, 1) * 2)).asMesh().rotated(q).translated(point).drawShadow().draw(Palette::Green);
		}
	}
	void addRigidBody(SivBlock& block)
	{
		auto rigidBodyPtr = block.getRigidBodyPtr();
		btRigidBodies.emplace_back(rigidBodyPtr);
		dynamicsWorld->addRigidBody(rigidBodyPtr);
	}
private:
	btDiscreteDynamicsWorld* dynamicsWorld;
	btSequentialImpulseConstraintSolver* solver;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btAxisSweep3* broadphase;
	btStaticPlaneShape* groundShape;
	btDefaultMotionState* groundMotionState;
	btRigidBody* groundRigidBody;
	btCollisionDispatcher* dispatcher;
	Array<btRigidBody*> btRigidBodies;
};

