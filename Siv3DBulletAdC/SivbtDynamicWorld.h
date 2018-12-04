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
	void update();
	void draw() const;
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

