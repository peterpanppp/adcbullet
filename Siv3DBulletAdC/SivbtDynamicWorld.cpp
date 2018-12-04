#include "SivbtDynamicWorld.h"



SivbtDynamicWorld::SivbtDynamicWorld()
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


SivbtDynamicWorld::~SivbtDynamicWorld()
{
	dynamicsWorld->removeRigidBody(groundRigidBody);
	delete groundRigidBody->getMotionState();
	delete groundRigidBody;

	//delete groundMotionState;
	delete solver;
	delete collisionConfiguration;
	delete broadphase;
	delete groundShape;
	delete dynamicsWorld;
	delete dispatcher;
}
