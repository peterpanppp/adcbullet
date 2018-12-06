# include "BulletHelper.hpp"

void nene_game_bthelper()
{
	Window::Resize(640 * 2, 480 * 2);
	Graphics::SetBackground(Color(172, 69, 124));
	s3dbt::btBox originBox(Vec3::Zero, Vec3::One, 0, Color(0, 0, 0, 0), false);
	s3dbt::btBox boardBox(Vec3(0, 3, 0), Vec3(10, 0.25, 10), 100000.0, Color(107, 150, 228));
	s3dbt::btDynamicWorld world;
	world.addRigidBody(originBox);
	world.addRigidBody(boardBox);
	s3dbt::btUniversalJoint uniJoint(originBox, boardBox, Vec3::Zero, Vec3::UnitZ, Vec3::UnitX);
	double range = 0.1;
	uniJoint.setAngularLowerLimit(Vec3(0, -Math::PiF * range, -Math::PiF * range));
	uniJoint.setAngularUpperLimit(Vec3(0, Math::PiF * range, Math::PiF * range));
	world.addJoint(uniJoint);
	uniJoint.setRotationalLimitMotor(2);
	uniJoint.setRotationalLimitMotor(1);
	s3dbt::btSphere ball(Vec3(8.5, 5, 8.5), 0.25, 1, Color(141, 144, 179));
	world.addRigidBody(ball);
	Array<s3dbt::btBox> walls;
	Array<s3dbt::bt2PointJoint> wallJoints;
	{
		CSVReader csv(L"../FieldEdit/App/resource_realscale.csv");
		auto size = static_cast<int>(csv.rows);
		const double blocksCenterY = 3. + 0.25 + 0.5;
		auto wallSize = size;
		walls.reserve(size);
		wallJoints.reserve(size * 2);
		for (auto i : step(size))
		{
			auto sizex = csv.get<double>(i, 0);
			auto sizey = csv.get<double>(i, 1);
			auto centerx = csv.get<double>(i, 2);
			auto centery = csv.get<double>(i, 3);
			walls.emplace_back(Vec3(centerx - 10., blocksCenterY, -centery + 10.), Vec3(sizex, 1, sizey) * 0.5, 1);
		}
		for (auto&& wall : walls)
		{
			world.addRigidBody(wall);
			auto boxSize = wall.getSizeSiv3d() * 2;
			auto boxCenter = wall.getPosSiv3d();
			btVector3 btHalfSize(boxSize.x * 0.5, boxSize.y * 0.5, boxSize.z * 0.5);

			const Vec3 plateSize = boardBox.getSizeSiv3d();
			const Vec3 plateCenter = boardBox.getPosSiv3d();
			// ベースから左下手前
			Vec3 pivotInA(boxCenter.x - btHalfSize.x(), plateSize.y, boxCenter.z - btHalfSize.z());
			// 壁の中心から左下手前
			Vec3 pivotInB(-btHalfSize.x(), -btHalfSize.y(), -btHalfSize.z());
			wallJoints.emplace_back(boardBox, wall, pivotInA, pivotInB);
			// ベースから右下奥
			Vec3 pivotInC(boxCenter.x + btHalfSize.x(), plateSize.y, boxCenter.z + btHalfSize.z());
			// 壁の中心から右下奥
			Vec3 pivotInD(btHalfSize.x(), -btHalfSize.y(), btHalfSize.z());
			wallJoints.emplace_back(boardBox, wall, pivotInC, pivotInD);
		}
		for (auto&& joint : wallJoints)
		{
			world.addJoint(joint);
		}
	}

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