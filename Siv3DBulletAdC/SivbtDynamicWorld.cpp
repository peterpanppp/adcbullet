#include "SivbtDynamicWorld.h"



SivbtDynamicWorld::SivbtDynamicWorld()
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
