#pragma once

namespace LCN
{
	template<class C1, class C2>
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
