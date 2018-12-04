#pragma once
#include<Siv3D.hpp>
#include<btBulletDynamicsCommon.h>

class Bullet
{
public:
	Bullet();
	~Bullet();
	void init()
	{
		// ワールドの広さ
		btVector3 worldAabbMin(-10000, -10000, -10000);
		btVector3 worldAabbMax(10000, 10000, 10000);
		// プロキシの最大数（衝突物体のようなもの）
		int maxProxies = 1024;
		// broadphaseの作成（SAP法）
		broadphase = std::make_shared<btAxisSweep3>(worldAabbMin, worldAabbMax, maxProxies);

		// デフォルトの衝突設定とディスパッチャの作成
		collisionConfiguration = std::make_shared<btDefaultCollisionConfiguration>();
		dispatcher = std::make_shared<btCollisionDispatcher>(collisionConfiguration.get());

		//// 衝突解決ソルバ
		solver = std::make_shared<btSequentialImpulseConstraintSolver>();

		dynamicsWorld = std::make_shared<btDiscreteDynamicsWorld>(dispatcher.get(), broadphase.get(), solver.get(), collisionConfiguration.get());

		dynamicsWorld->setGravity(btVector3(0, -10, 0));
	}
	void update()
	{
		dynamicsWorld->stepSimulation(1 / 60.f, 10);
	}
	void draw() const
	{
		btTransform trans;
		fallRigidBody->getMotionState()->getWorldTransform(trans);
		Plane(Vec3(0, 0, 0), 20).draw(Palette::Brown);
		Sphere(Vec3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()), 1).draw(Palette::Red);

	}
	void setGround()
	{
		// 地面の衝突形状の作成
		groundShape = std::make_shared<btStaticPlaneShape>(btVector3(0, 1, 0), 1);

		// 地面のMotionStateの設定
		groundMotionState =std::make_shared<btDefaultMotionState>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));

		// 地面の初期情報を設定
		btRigidBody::btRigidBodyConstructionInfo
			groundRigidBodyCI(0, groundMotionState.get(), groundShape.get(), btVector3(0, 0, 0));

		// 地面の剛体の作成
		groundRigidBody = std::make_shared<btRigidBody>(groundRigidBodyCI);
		// ワールドに地面の剛体を追加
		dynamicsWorld->addRigidBody(groundRigidBody.get());
	}
	void setSphere()
	{
		fallShape = std::make_shared<btSphereShape>(1);
		// 球のMotionStateの設定
		fallMotionState = std::make_shared<btDefaultMotionState>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 50, 0)));
		btScalar mass = 1;
		btVector3 fallInertia(0, 0, 0);
		fallShape->calculateLocalInertia(mass, fallInertia);
		btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState.get(), fallShape.get(), fallInertia);
		fallRigidBody = std::make_shared<btRigidBody>(fallRigidBodyCI);
		dynamicsWorld->addRigidBody(fallRigidBody.get());

	}
private:
	std::shared_ptr<btDiscreteDynamicsWorld> dynamicsWorld;
	std::shared_ptr<btRigidBody> fallRigidBody;
	std::shared_ptr<btAxisSweep3> broadphase;
	std::shared_ptr<btDefaultCollisionConfiguration> collisionConfiguration;
	std::shared_ptr<btCollisionDispatcher> dispatcher;
	std::shared_ptr<btSequentialImpulseConstraintSolver> solver;
	std::shared_ptr<btStaticPlaneShape> groundShape;
	std::shared_ptr<btDefaultMotionState> groundMotionState;
	std::shared_ptr<btRigidBody> groundRigidBody;
	std::shared_ptr<btSphereShape> fallShape;
	std::shared_ptr<btDefaultMotionState> fallMotionState;
};

