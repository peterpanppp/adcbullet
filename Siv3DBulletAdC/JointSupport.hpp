# pragma once
# include <Siv3D.hpp>

class JointSupport
{
public:
	JointSupport() :
		center(0, 0, 0), 
		m_pos(Vec3::Zero),
		points
		{
			Vec3(0.5, 0, 0) + center,
			Vec3(-0.5, 0, 0) + center,
			Vec3(0, 0, 0.5) + center
		}
	{

	}
	JointSupport(const Vec3& size) :
		center(0, 0, 0),
		m_pos(Vec3::Zero),
		points
		{
			Vec3(size.x * 0.5, 0, 0) + center,
			Vec3(-size.x * 0.5, 0, 0) + center,
			Vec3(0, 0, size.z * 0.5) + center
		}
	{

	}
	JointSupport(const Vec3& size, const Vec3& pos) :
		center(0, 0, 0), 
		m_pos(Vec3::Zero),
		points
		{
			Vec3(size.x * 0.5, 0, 0) + center,
			Vec3(-size.x * 0.5, 0, 0) + center,
			Vec3(0, 0, size.z * 0.5) + center
		}
	{

	}
	~JointSupport() = default;
	void update()
	{
		if (Input::KeyRight.pressed)
		{
			q.rotateRollPitchYaw(-delta, 0, 0);
			Quaternion rot(0, 0, 0, 1);
			rot.rotateRollPitchYaw(-delta, 0, 0);
			for (auto& pos : points)
			{
				pos = rot * pos;
			}
		}
		if (Input::KeyLeft.pressed)
		{
			q.rotateRollPitchYaw(delta, 0, 0);
			Quaternion rot(0, 0, 0, 1);
			rot.rotateRollPitchYaw(delta, 0, 0);
			for (auto& pos : points)
			{
				pos = rot * pos;
			}
		}
		if (Input::KeyUp.pressed)
		{
			q.rotateRollPitchYaw(0, delta, 0);
			Quaternion rot(0, 0, 0, 1);
			rot.rotateRollPitchYaw(0, delta, 0);
			for (auto& pos : points)
			{
				pos = rot * pos;
			}
		}
		if (Input::KeyDown.pressed)
		{
			q.rotateRollPitchYaw(0, -delta, 0);
			Quaternion rot(0, 0, 0, 1);
			rot.rotateRollPitchYaw(0, -delta, 0);
			for (auto& pos : points)
			{
				pos = rot * pos;
			}
		}
	}
	std::array<Vec3, 3> getPoints() const
	{
		std::array<Vec3, 3> returnPoints
		{
			points[0] + m_pos,
			points[1] + m_pos,
			points[2] + m_pos
		};
		return returnPoints;
	}
	Quaternion getQuaternion() const
	{
		return q;
	}
private:
	const Vec3 center;
	std::array<Vec3, 3> points;
	const double delta = 0.01;
	Quaternion q;
	Vec3 m_pos;
};

