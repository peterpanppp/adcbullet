# include "SivbtDynamicWorld.h"
# include "SivBlock.h"
# include "SivJoint.h"
# include "SivUniversalJoint.h"


void SivBlockTest()
{
	Window::Resize(640 * 2, 480 * 2);
	SivBlock sblock0(Vec3::One, Vec3::Zero, 0);
	SivBlock sblock1(Vec3::One, Vec3(2, 0, 0), 1);
	SivBlock sblock2(Vec3::One, Vec3(-2, 0, 0), 1);
	SivBlock sblock3(Vec3(10, 1, 10), Vec3(0, 10, 0), 1);
	SivbtDynamicWorld world;
	world.addRigidBody(sblock0);
	world.addRigidBody(sblock1);
	world.addRigidBody(sblock2);
	world.addRigidBody(sblock3);
	SivJoint joint(sblock0, sblock1);
	world.addJoint(joint);

	SivUniversalJoint uniJoint(sblock0, sblock3, Vec3::Zero, Vec3(0, 0, 1), Vec3(1, 0, 0));
	uniJoint.setAngularLowerLimit(Vec3(0, -Math::PiF * 0.05, -Math::PiF * 0.05));
	uniJoint.setAngularUpperLimit(Vec3(0, Math::PiF * 0.05, Math::PiF * 0.05));
	world.addJoint(uniJoint);

	while (System::Update())
	{
		world.update();
		world.draw();
	}
}