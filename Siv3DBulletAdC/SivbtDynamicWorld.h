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
# include "SivJoint.h"
# include "SivUniversalJoint.h"
# include "SivSphere.h"

class SivbtDynamicWorld
{
public:
	SivbtDynamicWorld()
	{
	#pragma region INIT_BULLET_WORLD

		// ���[���h�̍L��
		btVector3 worldAabbMin(-10000, -10000, -10000);
		btVector3 worldAabbMax(10000, 10000, 10000);
		// �v���L�V�̍ő吔�i�Փ˕��̂̂悤�Ȃ��́j
		int maxProxies = 1024;
		// broadphase�̍쐬�iSAP�@�j
		broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

		// �f�t�H���g�̏Փːݒ�ƃf�B�X�p�b�`���̍쐬
		collisionConfiguration = new btDefaultCollisionConfiguration();
		dispatcher = new btCollisionDispatcher(collisionConfiguration);

		// �Փˉ����\���o
		solver = new btSequentialImpulseConstraintSolver;

		// ���U���I���E�̍쐬
		dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

		// �d�͂̐ݒ�
		dynamicsWorld->setGravity(btVector3(0, -10, 0));

		// �n�ʂ̏Փˌ`��̍쐬
		groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

		// �n�ʂ�MotionState�̐ݒ�
		groundMotionState =
			new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -10, 0)));

		// �n�ʂ̏�������ݒ�
		btRigidBody::btRigidBodyConstructionInfo
			groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, -10, 0));

		// �n�ʂ̍��̂̍쐬
		groundRigidBody = new btRigidBody(groundRigidBodyCI);
		// ���[���h�ɒn�ʂ̍��̂�ǉ�
		dynamicsWorld->addRigidBody(groundRigidBody/*.get()*/);


		/*******************************************************/
#pragma endregion
	}
	~SivbtDynamicWorld()
	{
		dynamicsWorld->removeRigidBody(groundRigidBody);
		delete groundRigidBody->getMotionState();
		delete groundRigidBody;
		for (auto RigidBody : btRigidBodies)
		{
			dynamicsWorld->removeRigidBody(RigidBody);
			delete RigidBody->getMotionState();
			delete RigidBody;
		}
		for (auto RigidBody : btRigidSphereBodies)
		{
			dynamicsWorld->removeRigidBody(RigidBody);
			delete RigidBody->getMotionState();
			delete RigidBody;
		}
		delete solver;
		delete collisionConfiguration;
		delete broadphase;
		delete groundShape;
		delete dispatcher;
		delete dynamicsWorld;
	}
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
		assert(btRigidBodies.size() == blockHalfSizes.size());
		for (auto i : step(btRigidBodies.size()))
		{
			btTransform trans;
			btRigidBodies[i]->getMotionState()->getWorldTransform(trans);
			auto rot = trans.getRotation();
			Quaternion q(rot.getX(), rot.getY(), rot.getZ(), rot.getW());
			auto pos = trans.getOrigin();
			Vec3 point(pos.getX(), pos.getY(), pos.getZ());
			auto hsize = blockHalfSizes[i];
			Box(Vec3::Zero, bt2s3d(btVector3(hsize.x, hsize.y, hsize.z) * 2)).asMesh().rotated(q).translated(point).drawShadow().draw(Palette::Green);
		}
		assert(btRigidSphereBodies.size() == spheresRadius.size());
		for (auto i : step(btRigidSphereBodies.size()))
		{
			btTransform trans;
			btRigidSphereBodies[i]->getMotionState()->getWorldTransform(trans);
			auto rot = trans.getRotation();
			Quaternion q(rot.getX(), rot.getY(), rot.getZ(), rot.getW());
			auto pos = trans.getOrigin();
			Vec3 point(pos.getX(), pos.getY(), pos.getZ());
			auto radius = spheresRadius[i];
			Sphere(Vec3::Zero, radius).asMesh().rotated(q).translated(point).drawShadow().draw(Palette::Green);
		}
	}
	void addRigidBody(SivBlock& block)
	{
		blockHalfSizes.emplace_back(block.getSizeSiv3d());
		auto rigidBodyPtr = block.getRigidBodyPtr();
		btRigidBodies.emplace_back(rigidBodyPtr);
		dynamicsWorld->addRigidBody(rigidBodyPtr);
	}
	void addRigidBody(SivSphere& sphere)
	{
		spheresRadius.emplace_back(sphere.getRadius());
		auto rigidBodyPtr = sphere.getRigidBodyPtr();
		btRigidSphereBodies.emplace_back(rigidBodyPtr);
		dynamicsWorld->addRigidBody(rigidBodyPtr);
	}
	void addJoint(SivJoint& joint)
	{
		dynamicsWorld->addConstraint(joint.getbtPoint2PointConstraint());
	}
	void addJoint(SivUniversalJoint& joint)
	{
		dynamicsWorld->addConstraint(joint.getUniversalConstraintPtr());
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
	Array<btRigidBody*> btRigidSphereBodies;
	Array<Vec3> blockHalfSizes;
	Array<double> spheresRadius;
};

