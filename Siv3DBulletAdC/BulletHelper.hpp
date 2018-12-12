#pragma once

# include <Siv3D.hpp>
# include "BulletCollision/CollisionDispatch/btGhostObject.h"
# include "ConstraintDemo.h"
# include "btBulletDynamicsCommon.h"
# include "LinearMath/btIDebugDraw.h"

namespace s3dbt
{
struct btBoxData
{
	Vec3 halfSize;
	Vec3 center;
	Color color;
	bool drawFlg;
};

Vec3 ChangeHandedSystemVec3(const Vec3& vec)
{
	return Vec3(-vec.x, vec.y, vec.z);
}

Quaternion ChangeHandedSystemVec3(const Quaternion& quaternion)
{
	auto q = quaternion.component.m128_f32;
	return Quaternion(-q[0], q[1], q[2], -q[3]);
}

class btBox
{
public:
	btBox(const Vec3& boxCenter = Vec3::Zero, const Vec3& halfSize = Vec3::One, double weight = 0, Color color = Palette::White, bool drawFlg = true)
	{
		size = halfSize;
		box = new btBoxShape(btVector3(size.x, size.y, size.z));
		Vec3 iniBoxPos(boxCenter);
		center = iniBoxPos;
		fallMotionStateBox =
			new btDefaultMotionState(
				btTransform(btQuaternion(0, 0, 0, 1), btVector3(iniBoxPos.x, iniBoxPos.y, iniBoxPos.z)));
		mass = weight;
		btVector3 fallInertia(0, 0, 0);
		box->calculateLocalInertia(mass, fallInertia);
		btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCIBox(mass, fallMotionStateBox, box, fallInertia);
		fallRigidBodyBox = new btRigidBody(fallRigidBodyCIBox);
		fallRigidBodyBox->setActivationState(DISABLE_DEACTIVATION);
		data.center = center;
		data.halfSize = size;
		data.color = color;
		data.drawFlg = drawFlg;
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

class bt2PointJoint
{
public:
	bt2PointJoint(btBox& box0, btBox& box1)
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
	bt2PointJoint(btBox& box0, btBox& box1, const Vec3& pivot0, const Vec3& pivot1)
	{
		btVector3 pivotInA(pivot0.x, pivot0.y, pivot0.z);
		btVector3 pivotInB(pivot1.x, pivot1.y, pivot1.z);
		p2p = new btPoint2PointConstraint(*(box0.getRigidBodyPtr()), *(box1.getRigidBodyPtr()), pivotInA, pivotInB);
	}
	~bt2PointJoint()
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

class btUniversalJoint
{
public:
	btUniversalJoint(btBox& originBox, btBox& box, const Vec3& anchorPos = Vec3::Zero,
		const Vec3& axis0 = Vec3::Zero, const Vec3& axis1 = Vec3::Zero)
	{
		pUniv = new btUniversalConstraint(*(originBox.getRigidBodyPtr()), *(box.getRigidBodyPtr()),
			btVector3(anchorPos.x, anchorPos.y, anchorPos.z),
			btVector3(axis0.x, axis0.y, axis0.z),
			btVector3(axis1.x, axis1.y, axis1.z));
	}
	~btUniversalJoint()
	{
		delete pUniv;
	}
	/// <summary>
	/// 下限がマイナス範囲の場合はマイナス符号をつけてください
	/// </summary>
	void setAngularLowerLimit(const Vec3& lowerLimit)
	{
		pUniv->setAngularLowerLimit(btVector3(lowerLimit.x, lowerLimit.y, lowerLimit.z));
	}
	void setAngularUpperLimit(const Vec3& upperLimit)
	{
		pUniv->setAngularUpperLimit(btVector3(upperLimit.x, upperLimit.y, upperLimit.z));
	}
	btUniversalConstraint* getUniversalConstraintPtr()
	{
		return pUniv;
	}
	void setRotationalLimitMotor(int index)
	{
		motors.emplace(index, std::move(pUniv->getRotationalLimitMotor(index)));
		motors[index]->m_enableMotor = true;
		motors[index]->m_targetVelocity = btRadians(0.0);
	}
	/// <summary>
	/// motorを停止する場合は第2引数に0を入れてください
	/// </summary>
	void setMotorForce(int index, double rad, double force)
	{
		if (motors.count(index) == 0)
		{
			// 該当するindexがkeyのmotorが存在しません
			return;
		}
		motors[index]->m_targetVelocity = btRadians(rad);
		motors[index]->m_maxMotorForce = force;
	}
private:
	btUniversalConstraint* pUniv;
	std::map<int, btRotationalLimitMotor*> motors;
};

struct btSphereData
{
	Vec3 center;
	double radius;
	Color color;
};

class btSphere
{
public:
	btSphere(const Vec3& center = Vec3::Zero, double ballRadius = 1, double mass = 1, Color color = Palette::White) :
		radius(ballRadius)
	{
		pColShape = new btSphereShape(btScalar(radius));
		fallMotionStateBall =
			new btDefaultMotionState(
				btTransform(btQuaternion(0, 0, 0, 1), btVector3(center.x, center.y, center.z)));
		btVector3 fallInertia(0, 0, 0);
		pColShape->calculateLocalInertia(mass, fallInertia);
		btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCIDynamicBall
		(mass, fallMotionStateBall, pColShape, fallInertia);
		fallRigidBodyBall = new btRigidBody(fallRigidBodyCIDynamicBall);
		data.center = center;
		data.radius = ballRadius;
		data.color = color;
	}
	~btSphere()
	{
		delete pColShape;
	}
	double getRadius() const
	{
		return radius;
	}
	btRigidBody* getRigidBodyPtr()
	{
		return fallRigidBodyBall;
	}
	btSphereData getData() const
	{
		return data;
	}
private:
	btSphereShape* pColShape;
	btDefaultMotionState* fallMotionStateBall;
	btRigidBody* fallRigidBodyBall;
	double radius;
	btSphereData data;
};

class btDynamicWorld
{
public:
	btDynamicWorld()
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
	~btDynamicWorld()
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
		assert(btRigidBodies.size() == blockDatas.size());
		for (auto i : step(btRigidBodies.size()))
		{
			auto blockData = blockDatas[i];
			if (!blockData.drawFlg)
			{
				continue;
			}
			btTransform trans;
			btRigidBodies[i]->getMotionState()->getWorldTransform(trans);
			auto rot = trans.getRotation();
			Quaternion q(rot.getX(), rot.getY(), rot.getZ(), rot.getW());
			auto pos = trans.getOrigin();
			Vec3 point(pos.getX(), pos.getY(), pos.getZ());
			auto hsize = blockData.halfSize;
			Box(Vec3::Zero, hsize * 2).asMesh().rotated(q).translated(point).drawShadow().draw(blockData.color);
		}
		assert(btRigidSphereBodies.size() == sphereDatas.size());
		for (auto i : step(btRigidSphereBodies.size()))
		{
			btTransform trans;
			btRigidSphereBodies[i]->getMotionState()->getWorldTransform(trans);
			auto rot = trans.getRotation();
			Quaternion q(rot.getX(), rot.getY(), rot.getZ(), rot.getW());
			auto pos = trans.getOrigin();
			Vec3 point(pos.getX(), pos.getY(), pos.getZ());
			auto radius = sphereDatas[i].radius;
			Sphere(Vec3::Zero, radius).asMesh().rotated(q).translated(point).drawShadow().draw(sphereDatas[i].color);
		}
	}
	void addRigidBody(btBox& block)
	{
		auto rigidBodyPtr = block.getRigidBodyPtr();
		btRigidBodies.emplace_back(rigidBodyPtr);
		dynamicsWorld->addRigidBody(rigidBodyPtr);
		blockDatas.emplace_back(block.getData());
	}
	void addRigidBody(btSphere& sphere)
	{
		auto rigidBodyPtr = sphere.getRigidBodyPtr();
		btRigidSphereBodies.emplace_back(rigidBodyPtr);
		dynamicsWorld->addRigidBody(rigidBodyPtr);
		sphereDatas.emplace_back(sphere.getData());
	}
	void addJoint(bt2PointJoint& joint)
	{
		dynamicsWorld->addConstraint(joint.getbtPoint2PointConstraint());
	}
	void addJoint(btUniversalJoint& joint)
	{
		dynamicsWorld->addConstraint(joint.getUniversalConstraintPtr());
	}
	void addJoint(btUniversalConstraint* pUniv)
	{
		dynamicsWorld->addConstraint(pUniv);
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
	Array<btBoxData> blockDatas;
	Array<btSphereData> sphereDatas;
};
} // s3dbt
