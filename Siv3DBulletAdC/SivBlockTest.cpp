# include "SivbtDynamicWorld.h"
# include "SivBlock.h"
# include "SivJoint.h"
# include "SivUniversalJoint.h"
# include "SivSphere.h"


void SivBlockTest()
{
	Window::Resize(640 * 2, 480 * 2);
	btBox sblock0(Vec3::One, Vec3::Zero, 0);
	btBox sblock1(Vec3::One, Vec3(2, 0, 0), 1);
	btBox sblock2(Vec3::One, Vec3(-2, 0, 0), 1);
	btBox sblock3(Vec3(10, 1, 10), Vec3(0, 10, 0), 1);
	btDynamicWorld world;
	world.addRigidBody(sblock0);
	world.addRigidBody(sblock1);
	world.addRigidBody(sblock2);
	world.addRigidBody(sblock3);
	bt2PointJoint joint(sblock0, sblock1);
	world.addJoint(joint);

	btUniversalJoint uniJoint(sblock0, sblock3, Vec3::Zero, Vec3(0, 0, 1), Vec3(1, 0, 0));
	uniJoint.setAngularLowerLimit(Vec3(0, -Math::PiF * 0.05, -Math::PiF * 0.05));
	uniJoint.setAngularUpperLimit(Vec3(0, Math::PiF * 0.05, Math::PiF * 0.05));
	world.addJoint(uniJoint);
	uniJoint.setRotationalLimitMotor(2);
	uniJoint.setRotationalLimitMotor(1);

	btSphere sphere(Vec3(0, 5, 0), 0.25, 1);
	world.addRigidBody(sphere);

	while (System::Update())
	{
		double force = 100000.;
		if (Input::KeyG.pressed)
		{
			uniJoint.setMotorForce(2, -10, force);
		}
		else if (Input::KeyH.pressed)
		{
			uniJoint.setMotorForce(2, 10, force);
		}
		else
		{
			uniJoint.setMotorForce(2, 0, force);
		}
		if (Input::KeyY.pressed)
		{
			uniJoint.setMotorForce(1, -10, force);
		}
		else if (Input::KeyB.pressed)
		{
			uniJoint.setMotorForce(1, 10, force);
		}
		else
		{
			uniJoint.setMotorForce(1, 0, force);
		}
		world.update();
		world.draw();
	}
}