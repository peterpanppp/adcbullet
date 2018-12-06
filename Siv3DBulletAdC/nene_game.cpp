# include "SivbtDynamicWorld.h"

void nene_game()
{
	SivBlock originBox;
	SivBlock boardBox(Vec3(10, 0.25, 10), Vec3(0, 3, 0), 100000.0);
	SivbtDynamicWorld world;
	world.addRigidBody(originBox);
	world.addRigidBody(boardBox);
	SivUniversalJoint uniJoint(originBox, boardBox, Vec3::Zero, Vec3::UnitZ, Vec3::UnitX);
	double range = 0.1;
	uniJoint.setAngularLowerLimit(Vec3(0, -Math::PiF * range, -Math::PiF * range));
	uniJoint.setAngularUpperLimit(Vec3(0, Math::PiF * range, Math::PiF * range));
	world.addJoint(uniJoint);
	uniJoint.setRotationalLimitMotor(2);
	uniJoint.setRotationalLimitMotor(1);
	SivSphere ball(Vec3(0, 5, 0), 0.25);
	world.addRigidBody(ball);

	while (System::Update())
	{
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
		}

		world.update();
		world.draw();
	}
}