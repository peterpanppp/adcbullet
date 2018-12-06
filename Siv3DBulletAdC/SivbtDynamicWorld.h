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

		// ワールドの広さ
		btVector3 worldAabbMin(-10000, -10000, -10000);
		btVector3 worldAabbMax(10000, 10000, 10000);
		// プロキシの最大数（衝突物体のようなもの）
		int maxProxies = 1024;
		// broadphaseの作成（SAP法）
		broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

		// デフォルトの衝突設定とディスパッチャの作成
		collisionConfiguration = new btDefaultCollisionConfiguration();
		dispatcher = new btCollisionDispatcher(collisionConfiguration);

		// 衝突解決ソルバ
		solver = new btSequentialImpulseConstraintSolver;

		// 離散動的世界の作成
		dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

		// 重力の設定
		dynamicsWorld->setGravity(btVector3(0, -10, 0));

		// 地面の衝突形状の作成
		groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

		// 地面のMotionStateの設定
		groundMotionState =
			new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -10, 0)));

		// 地面の初期情報を設定
		btRigidBody::btRigidBodyConstructionInfo
			groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, -10, 0));

		// 地面の剛体の作成
		groundRigidBody = new btRigidBody(groundRigidBodyCI);
		// ワールドに地面の剛体を追加
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

