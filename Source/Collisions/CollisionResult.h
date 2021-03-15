#pragma once

namespace LCN
{
	template<class Shape1, class Shape2>
	class CollisionResult
	{
	public:
		CollisionResult(bool collide) :
			m_Collide(collide)
		{}

		operator bool() const { return m_Collide; }

	private:
		bool m_Collide = false;
	};
}
