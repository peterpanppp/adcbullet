
# include <Siv3D.hpp>
# include "BulletCollision/CollisionDispatch/btGhostObject.h"
# include "Block.h"
# include "BlockJoint.h"
# include "JointSupport.hpp"
# include "ConstraintDemo.h"
# include "btBulletDynamicsCommon.h"
# include "LinearMath/btIDebugDraw.h"
/* とりあえずラップとかはせずにテスト */
namespace
{

btVector3 bt2s3d(const Vec3& vec)
{
	return btVector3(vec.x, vec.y, vec.z);
}

double Sine0_1(double periodSec, double t = Time::GetMicrosec() / 1'000'000.0)
{
	const double x = std::fmod(t, periodSec) / (periodSec * (1.0 / Math::TwoPi));

	return std::sin(x) * 0.5 + 0.5;
}

btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape, btDiscreteDynamicsWorld* ownerWorld)
{
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	ownerWorld->addRigidBody(body);

	return body;
}

btPoint2PointConstraint* createJoint(const Vec3& ghostPos, const Vec3& boardPos, std::shared_ptr<Block> boardBlock, std::shared_ptr<Block> ghost, btDiscreteDynamicsWorld* dynamicsWorld)
{
	Vec3 centerBoard;
	btPoint2PointConstraint* p2pBoard;
	{
		centerBoard = (ghostPos + boardPos) * 0.5;
		Vec3 chainBoard = centerBoard - boardPos;
		Vec3 chainGhost = centerBoard - ghostPos;
		btVector3 pivotInA(chainBoard.x, chainBoard.y, chainBoard.z);
		btVector3 pivotInB(chainGhost.x, chainGhost.y, chainGhost.z);
		p2pBoard = new btPoint2PointConstraint(*(boardBlock->getBoxRigidBodyPtr()), *(ghost->getBoxRigidBodyPtr()), pivotInA, pivotInB);
		dynamicsWorld->addConstraint(p2pBoard);
	}
	return p2pBoard;
}

void DrawBoxBody(btRigidBody* body, const Vec3& halfSize, const Color& color = Palette::White)
{
	btTransform Trans;
	body->getMotionState()->getWorldTransform(Trans);
	auto rot = Trans.getRotation();
	Quaternion qb(rot.getX(), rot.getY(), rot.getZ(), rot.getW());
	auto origin = Trans.getOrigin();
	Vec3 pos(origin.getX(), origin.getY(), origin.getZ());
	Box(Vec3::Zero, halfSize * 2).asMesh().rotated(qb).translated(pos).draw(color).drawShadow();
}

void DrawBaseBody(btRigidBody* body, const Vec3& halfSize, const Color& color = Palette::White)
{
	btTransform Trans;
	body->getMotionState()->getWorldTransform(Trans);
	auto rot = Trans.getRotation();
	Quaternion qb(rot.getX(), rot.getY(), rot.getZ(), rot.getW());
	auto origin = Trans.getOrigin();
	Vec3 pos(origin.getX(), origin.getY(), origin.getZ());
	Plane(pos + Vec3(0, 0.26, 0), 19.9, qb).draw(color);
	Box(Vec3::Zero, halfSize * 2).asMesh().rotated(qb).translated(pos).draw(Palette::White).drawShadow();
}

void DrawBoxSphere(btRigidBody* body, double radius, const Color& color = Palette::White)
{
	btTransform sTrans;
	body->getMotionState()->getWorldTransform(sTrans);
	auto origin = sTrans.getOrigin();
	Vec3 pos(origin.getX(), origin.getY(), origin.getZ());
	Sphere(pos, radius).drawShadow().draw(color);
}

btRigidBody* CreateWall(const Box& box, btRigidBody* basePlate, btDiscreteDynamicsWorld* dynamicsWorld)
{
	auto boxSize = box.size;
	auto boxCenter = box.center;
	btVector3 btHalfSize(boxSize.x * 0.5, boxSize.y * 0.5, boxSize.z * 0.5);
	auto dynamicBox = new btBoxShape(btHalfSize);
	btVector3 fallInertia(0, 0, 0);
	const double mass = 1;
	dynamicBox->calculateLocalInertia(mass, fallInertia);
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(s3d2bt(boxCenter));
	auto partitionBody = localCreateRigidBody(mass, tr, dynamicBox, dynamicsWorld);

	const Vec3 plateCenter = Vec3::Zero;
	const Vec3 plateSize = Vec3(10, 0.25, 10);
	// ベースから左下手前
	btVector3 pivotInA(boxCenter.x - btHalfSize.x(), plateSize.y, boxCenter.z - btHalfSize.z());
	// 壁の中心から左下手前
	btVector3 pivotInB(-btHalfSize.x(), -btHalfSize.y(), -btHalfSize.z());
	btTypedConstraint* p2p0 = new btPoint2PointConstraint(*basePlate, *partitionBody, pivotInA, pivotInB);
	dynamicsWorld->addConstraint(p2p0);

	// ベースから右下奥
	btVector3 pivotInC(boxCenter.x + btHalfSize.x(), plateSize.y, boxCenter.z + btHalfSize.z());
	// 壁の中心から右下奥
	btVector3 pivotInD(btHalfSize.x(), -btHalfSize.y(), btHalfSize.z());
	btTypedConstraint* p2p1 = new btPoint2PointConstraint(*basePlate, *partitionBody, pivotInC, pivotInD);
	dynamicsWorld->addConstraint(p2p1);

	return partitionBody;
}

btRigidBody* CreateGoal(const Box& box, btRigidBody* basePlate, btDiscreteDynamicsWorld* dynamicsWorld)
{
	auto boxSize = box.size;
	auto boxCenter = box.center;
	btVector3 btHalfSize(boxSize.x * 0.5, boxSize.y * 0.5, boxSize.z * 0.5);
	auto dynamicBox = new btBoxShape(btHalfSize);
	btVector3 fallInertia(0, 0, 0);
	const double mass = 1;
	dynamicBox->calculateLocalInertia(mass, fallInertia);
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(s3d2bt(boxCenter));
	auto partitionBody = localCreateRigidBody(mass, tr, dynamicBox, dynamicsWorld);

	const Vec3 plateCenter = Vec3::Zero;
	const Vec3 plateSize = Vec3(10, 0.25, 10);
	// ベースから左下手前
	btVector3 pivotInA(boxCenter.x - btHalfSize.x(), plateSize.y, boxCenter.z - btHalfSize.z());
	// 壁の中心から左下手前
	btVector3 pivotInB(-btHalfSize.x(), -btHalfSize.y(), -btHalfSize.z());
	btTypedConstraint* p2p0 = new btPoint2PointConstraint(*basePlate, *partitionBody, pivotInA, pivotInB);
	dynamicsWorld->addConstraint(p2p0);

	// ベースから右下奥
	btVector3 pivotInC(boxCenter.x + btHalfSize.x(), plateSize.y, boxCenter.z + btHalfSize.z());
	// 壁の中心から右下奥
	btVector3 pivotInD(btHalfSize.x(), -btHalfSize.y(), btHalfSize.z());
	btTypedConstraint* p2p1 = new btPoint2PointConstraint(*basePlate, *partitionBody, pivotInC, pivotInD);
	dynamicsWorld->addConstraint(p2p1);

	return partitionBody;
}

void CreateStage(const Array<Box>& boxes, btRigidBody* basePlateBody, btDiscreteDynamicsWorld* dynamicsWorld, Array<btRigidBody*>& walls)
{
	for (const auto& box : boxes)
	{
		auto tmpBody = CreateWall(box, basePlateBody, dynamicsWorld);
		walls.emplace_back(tmpBody);
	}
}
}

void universal()
{
#pragma region INIT_BULLET_WORLD

	// ワールドの広さ
	btVector3 worldAabbMin(-10000, -10000, -10000);
	btVector3 worldAabbMax(10000, 10000, 10000);
	// プロキシの最大数（衝突物体のようなもの）
	int maxProxies = 1024;
	// broadphaseの作成（SAP法）
	btAxisSweep3* broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

	// デフォルトの衝突設定とディスパッチャの作成
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	auto dispatcher = new btCollisionDispatcher(collisionConfiguration);

	// 衝突解決ソルバ
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	// 離散動的世界の作成
	auto dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

	// 重力の設定
	dynamicsWorld->setGravity(btVector3(0, -10, 0));

	// 地面の衝突形状の作成
	auto groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

	// 地面のMotionStateの設定
	auto groundMotionState =
		new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -10, 0)));

	// 地面の初期情報を設定
	btRigidBody::btRigidBodyConstructionInfo
		groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, -10, 0));

	// 地面の剛体の作成
	auto groundRigidBody = new btRigidBody(groundRigidBodyCI);
	// ワールドに地面の剛体を追加
	dynamicsWorld->addRigidBody(groundRigidBody/*.get()*/);


	/*******************************************************/
#pragma endregion

	// 落下する球の衝突形状の作成

	btScalar mass = 1;
	btScalar massZero = 0;
	auto dynamicBox = new btBoxShape(btVector3(1, 1, 1));
	auto boardBox = new btBoxShape(btVector3(10, 0.25, 10));
	auto pColShape = new btSphereShape(btScalar(0.25));
	auto partitionBox = new btBoxShape(btVector3(1, 1, 1));
	
	btDefaultMotionState* fallMotionStateBall =
		new btDefaultMotionState(
			btTransform(btQuaternion(0, 0, 0, 1), bt2s3d(Vec3(8.5, 10, 8.5))));

	btVector3 fallInertia(0, 0, 0);
	boardBox->calculateLocalInertia(mass, fallInertia);
	pColShape->calculateLocalInertia(mass, fallInertia);
	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCIDynamicBall
	(mass, fallMotionStateBall, pColShape, fallInertia);

	auto fallRigidBodydynamicBall = new btRigidBody(fallRigidBodyCIDynamicBall);
	dynamicsWorld->addRigidBody(fallRigidBodydynamicBall);

	btTransform tr;
	tr.setIdentity();

	tr.setOrigin(btVector3(btScalar(0.), btScalar(0.), btScalar(0.)));
	btRigidBody* originBody = localCreateRigidBody(0, tr, dynamicBox, dynamicsWorld);

	tr.setOrigin(btVector3(btScalar(0.), btScalar(3.), btScalar(0.)));
	btRigidBody* basePlateBody = localCreateRigidBody(100000.0, tr, boardBox, dynamicsWorld);

	originBody->setActivationState(DISABLE_DEACTIVATION);
	basePlateBody->setActivationState(DISABLE_DEACTIVATION);

	btVector3 anchor(0.0f, 0.0f, 0.0f);
	btVector3 btAxisA(0.0f, 0.0f, 1.0f);
	btVector3 btAxisB(1.0f, 0.0f, 0.0f);

	btUniversalConstraint* pUniv = new btUniversalConstraint(*originBody, *basePlateBody, anchor, btAxisA, btAxisB);
	dynamicsWorld->addConstraint(pUniv);
	pUniv->setAngularLowerLimit(btVector3(0, -SIMD_PI * 0.05, -SIMD_PI * 0.05));
	pUniv->setAngularUpperLimit(btVector3(0, SIMD_PI * 0.05, SIMD_PI * 0.05));

	auto motor0 = pUniv->getRotationalLimitMotor(2);
	motor0->m_enableMotor = true;
	motor0->m_targetVelocity = btRadians(0.0);
	auto motor1 = pUniv->getRotationalLimitMotor(1);
	motor1->m_enableMotor = true;
	motor1->m_targetVelocity = btRadians(0.0);

	const double blocksCenterY = 3. + 0.25 + 0.5;
	Array<Box> goals;
	Array<Box> boxes;
	{
		// 後ろから4つはgoal用
		CSVReader csv(L"../FieldEdit/App/resource.csv");
		auto size = static_cast<int>(csv.rows);
		for (auto i : step(size - 4))
		{
			const double scale = 20. / 500;
			auto centerx = csv.get<double>(i, 0) * scale;
			auto centery = csv.get<double>(i, 1) * scale;
			auto sizex = csv.get<double>(i, 2) * scale;
			auto sizey = csv.get<double>(i, 3) * scale;
			boxes.emplace_back(Box(Vec3(centerx - 10., blocksCenterY, -centery + 10.), Vec3(sizex, 1, sizey)));
		}
		for (auto i : step(4))
		{
			const double scale = 20. / 500;
			auto centerx = csv.get<double>(i + size - 4, 0) * scale;
			auto centery = csv.get<double>(i + size - 4, 1) * scale;
			auto sizex = csv.get<double>(i + size - 4, 2) * scale;
			auto sizey = csv.get<double>(i + size - 4, 3) * scale;
			if (i == 3)
			{
				goals.emplace_back(Box(Vec3(centerx - 10., blocksCenterY - 0.87, -centery + 10.), Vec3(sizex, 0.25, sizey)));
			}
			else
			{
				goals.emplace_back(Box(Vec3(centerx - 10., blocksCenterY - 0.75, -centery + 10.), Vec3(sizex, 0.5, sizey)));
			}
		}
	}
	Array<btRigidBody*> walls;
	CreateStage(boxes, basePlateBody, dynamicsWorld, walls);
	
	Graphics::SetBackground(Color(172, 69, 124));
	const Texture textureGoal(L"goal.png", TextureDesc::For3D);

	while (System::Update())
	{
		Graphics3D::FreeCamera();
		dynamicsWorld->stepSimulation(1 / 60.f, 10);
		{
			Line3D(Vec3(1000, 0, 0), Vec3(-1000, 0, 0)).drawForward(Palette::Red);
			Line3D(Vec3(0, 1000, 0), Vec3(0, -1000, 0)).drawForward(Palette::Blue);
			Line3D(Vec3(0, 0, 1000), Vec3(0, 0, -1000)).drawForward(Palette::Green);
		}
		DrawBaseBody(basePlateBody, Vec3(10, 0.25, 10), Color(107, 150, 228));
		DrawBoxSphere(fallRigidBodydynamicBall, 0.25, Color(141, 144, 179));
		for (auto i :step(walls.size()))
		{
			DrawBoxBody(walls[i], boxes[i].size * 0.5);
		}
		if (Input::KeySpace.pressed)
		{
			btTransform tf;
			fallRigidBodydynamicBall->getMotionState()->getWorldTransform(tf);
			auto pos = tf.getOrigin();
			auto arrowEnd = Vec3(pos.x(), pos.y(), pos.z());
			auto arrowStt = Vec3(pos.x(), pos.y() + 100, pos.z());
			Line3D(arrowStt, arrowEnd).drawForward();
		}
		{
			btTransform tf;
			basePlateBody->getMotionState()->getWorldTransform(tf);
			auto rot = tf.getRotation();
			Quaternion qb(rot.getX(), rot.getY(), rot.getZ(), rot.getW());
			for (const auto& box : goals)
			{
				auto center = box.center;
				Box(Vec3::Zero, box.size).asMesh().translated(center).rotated(qb).draw(Palette::White).drawShadow();
			}
			Plane(Vec3::Zero, Vec2(goals[3].size.x, goals[3].size.z)).asMesh()
				.translated(goals[3].center + Vec3(0, goals[3].size.y * 0.5 + 0.1, 0))
				.rotated(qb)
				.draw(textureGoal);
		}

#pragma region MOVE_THE_MOTOR
		double val = 1.;
		double force = 100000.;
		if (Input::KeyShift.pressed)
		{
			val = 100.;
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

	}

	dynamicsWorld->removeRigidBody(groundRigidBody);
	delete groundRigidBody->getMotionState();
	delete groundRigidBody;

	delete groundShape;

	delete dynamicsWorld;
	delete solver;
	delete dispatcher;
	delete collisionConfiguration;
	delete broadphase;

}
