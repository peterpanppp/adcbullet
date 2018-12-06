# include "SivbtDynamicWorld.h"

void nene_game()
{
	SivBlock originBox;
	SivBlock boardBox(Vec3(10, 0.25, 10), Vec3(0, 3, 0), 1);
	SivbtDynamicWorld world;
	world.addRigidBody(originBox);
	world.addRigidBody(boardBox);

	while (System::Update())
	{
		world.update();
		world.draw();
	}
}