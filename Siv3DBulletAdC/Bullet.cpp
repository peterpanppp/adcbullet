#include "Bullet.h"



Bullet::Bullet()
{
}


Bullet::~Bullet()
{
	dynamicsWorld->removeRigidBody(groundRigidBody.get());
	dynamicsWorld->removeRigidBody(fallRigidBody.get());
}
