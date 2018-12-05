# include <Siv3D.hpp>
# include "BulletCollision/CollisionDispatch/btGhostObject.h"
# include "Block.h"
# include "BlockJoint.h"
# include "JointSupport.hpp"
# include "ConstraintDemo.h"
# include "btBulletDynamicsCommon.h"
# include "LinearMath/btIDebugDraw.h"

# include "SivbtDynamicWorld.h"
# include "SivBlock.h"

void SivBlockTest()
{
	Block block(Vec3(1, 1, 1), Vec3::Zero, 10);
	SivBlock sblock0;
	SivBlock sblock1;
	SivBlock sblock2;
	SivbtDynamicWorld world;
	world.addRigidBody(sblock0);
	world.addRigidBody(sblock1);
	world.addRigidBody(sblock2);

	while (System::Update())
	{
		world.update();
		world.draw();
	}
}