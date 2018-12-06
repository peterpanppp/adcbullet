# include "SivbtDynamicWorld.h"

void nene_game()
{
	SivBlock originBox;
	SivBlock boardBox(Vec3(10, 0.25, 10), Vec3(0, 3, 0), 1);
	SivbtDynamicWorld world;
	world.addRigidBody(originBox);
	world.addRigidBody(boardBox);
	/*SivUniversalJoint uniJoint(originBox, boardBox, Vec3::Zero, Vec3::UnitZ, Vec3::UnitX);
	double range = 0.1;
	uniJoint.setAngularLowerLimit(Vec3(0, -Math::PiF * range, -Math::PiF * range));
	uniJoint.setAngularUpperLimit(Vec3(0, Math::PiF * range, Math::PiF * range));
	world.addJoint(uniJoint);
	uniJoint.setRotationalLimitMotor(2);
	uniJoint.setRotationalLimitMotor(1);*/
	btUniversalConstraint* pUniv = new btUniversalConstraint(*originBox.getRigidBodyPtr(), *boardBox.getRigidBodyPtr(), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(1, 0, 0));
	world.addJoint(pUniv);
	pUniv->setAngularLowerLimit(btVector3(0, -SIMD_PI * 0.05, -SIMD_PI * 0.05));
	pUniv->setAngularUpperLimit(btVector3(0, SIMD_PI * 0.05, SIMD_PI * 0.05));
	auto motor0 = pUniv->getRotationalLimitMotor(2);
	motor0->m_enableMotor = true;
	motor0->m_targetVelocity = btRadians(0.0);
	auto motor1 = pUniv->getRotationalLimitMotor(1);
	motor1->m_enableMotor = true;
	motor1->m_targetVelocity = btRadians(0.0);

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
		/*{
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
		}*/
#pragma region MOVE_THE_MOTOR
		double force = 100000.;
		if (Input::KeyShift.pressed)
		{
			force = 1000000.;
		}
		if (Input::KeyG.pressed)
		{
			motor0->m_targetVelocity = btRadians(-10);
			motor0->m_maxMotorForce = force;
		}
		else if (Input::KeyH.pressed)
		{
			motor0->m_targetVelocity = btRadians(10);
			motor0->m_maxMotorForce = force;
		}
		else
		{
			motor0->m_targetVelocity = btRadians(0);
		}
		if (Input::KeyY.pressed)
		{
			motor1->m_targetVelocity = btRadians(-10);
			motor1->m_maxMotorForce = force;
		}
		else if (Input::KeyB.pressed)
		{
			motor1->m_targetVelocity = btRadians(10);
			motor1->m_maxMotorForce = force;
		}
		else
		{
			motor1->m_targetVelocity = btRadians(0);
		}
#pragma endregion
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