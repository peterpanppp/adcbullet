# include "SivbtDynamicWorld.h"

void nene_game()
{
	SivBlock originBox;
	SivBlock boardBox(Vec3(10, 0.25, 10), Vec3(0, 3, 0), 1);
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

	Array<Rect> buttons
	{
		Rect(Point(500, 370), Point(40, 20)),
		Rect(Point(500, 400), Point(40, 20)),
		Rect(Point(500, 430), Point(40, 20)),
		Rect(Point(500, 460), Point(40, 20))
	};
	while (System::Update())
	{
		for (auto button : buttons)
		{
			button.draw(Palette::Yellow);
		}
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
		/*{
			double force = 100000.;
			if (Input::KeySpace.pressed)
			{
				force *= 100;
			}
			if (buttons[0].leftPressed)
			{
				uniJoint.setMotorForce(2, -10, force);
			}
			else if (buttons[1].leftPressed)
			{
				uniJoint.setMotorForce(2, 10, force);
			}
			else
			{
				uniJoint.setMotorForce(2, 0, force);
			}
			if (buttons[2].leftPressed)
			{
				uniJoint.setMotorForce(1, -10, force);
			}
			else if (buttons[3].leftPressed)
			{
				uniJoint.setMotorForce(1, 10, force);
			}
			else
			{
				uniJoint.setMotorForce(1, 0, force);
			}
		}*/
		world.update();
		world.draw();
	}
}