# include <Siv3D.hpp>
//# include "BulletCollision/CollisionDispatch/btGhostObject.h"
//# include "Block.h"
//# include "BlockJoint.h"
//# include "JointSupport.hpp"
//# include "ConstraintDemo.h"
//# include "btBulletDynamicsCommon.h"
//# include "LinearMath/btIDebugDraw.h"

# include "SivbtDynamicWorld.h"
# include "SivBlock.h"
# include "SivJoint.h"


void SivBlockTest()
{
	Window::Resize(640 * 2, 480 * 2);
	SivBlock sblock0(Vec3::One, Vec3::Zero, 0);
	SivBlock sblock1(Vec3::One, Vec3(2, 0, 0), 1);
	SivBlock sblock2(Vec3::One, Vec3(-2, 0, 0), 1);
	SivbtDynamicWorld world;
	world.addRigidBody(sblock0);
	world.addRigidBody(sblock1);
	world.addRigidBody(sblock2);
	SivJoint joint(sblock0, sblock1);
	world.addJoint(joint);
	
	while (System::Update())
	{
		world.update();
		world.draw();
	}
}