#pragma once
# include <Siv3D.hpp>
# include "BulletCollision/CollisionDispatch/btGhostObject.h"
# include "Block.h"
# include "BlockJoint.h"
# include "JointSupport.hpp"
# include "ConstraintDemo.h"
# include "btBulletDynamicsCommon.h"
# include "LinearMath/btIDebugDraw.h"
# include "SivBlock.h"

class SivUniversalJoint
{
public:
	SivUniversalJoint(SivBlock& originBox, SivBlock& box, const Vec3& anchorPos = Vec3::Zero,
		const Vec3& axis0 = Vec3::Zero, const Vec3& axis1 = Vec3::Zero)
	{
		pUniv = new btUniversalConstraint(*(originBox.getRigidBodyPtr()), *(box.getRigidBodyPtr()),
			btVector3(anchorPos.x, anchorPos.y, anchorPos.z), 
			btVector3(axis0.x, axis0.y, axis0.z), 
			btVector3(axis1.x, axis1.y, axis1.z));
	}
	~SivUniversalJoint()
	{
		delete pUniv;
	}
	/// <summary>
	/// 下限がマイナス範囲の場合はマイナス符号をつけてください
	/// </summary>
	void setAngularLowerLimit(const Vec3& lowerLimit)
	{
		pUniv->setAngularLowerLimit(btVector3(lowerLimit.x, lowerLimit.y, lowerLimit.z));
	}
	void setAngularUpperLimit(const Vec3& upperLimit)
	{
		pUniv->setAngularUpperLimit(btVector3(upperLimit.x, upperLimit.y, upperLimit.z));
	}
	btUniversalConstraint* getUniversalConstraintPtr()
	{
		return pUniv;
	}
	void setRotationalLimitMotor(int index)
	{
		motors.emplace(index, std::move(pUniv->getRotationalLimitMotor(index)));
		motors[index]->m_enableMotor = true;
		motors[index]->m_targetVelocity = btRadians(0.0);
	}
	/// <summary>
	/// motorを停止する場合は第2引数に0を入れてください
	/// </summary>
	void setMotorForce(int index, double rad, double force)
	{
		if (motors.count(index) == 0)
		{
			// 該当するindexがkeyのmotorが存在しません
			return;
		}
		motors[index]->m_targetVelocity = btRadians(rad);
		motors[index]->m_maxMotorForce = force;
	}
private:
	btUniversalConstraint* pUniv;
	std::map<int, btRotationalLimitMotor*> motors;
};

