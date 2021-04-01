#pragma once

#include <type_traits>

#include "LCN_Collisions/Source/Shapes/Plane.h"
#include "LCN_Collisions/Source/Shapes/Line.h"

namespace LCN
{
	class CollisionResultBase
	{
	public:
		inline CollisionResultBase(bool collide) :
			m_Collide(collide)
		{}

		inline operator bool() const { return m_Collide; }

	private:
		bool m_Collide = false;
	};

	template<class Shape1, class Shape2>
	class CollisionResult;

	template<typename T>
	class CollisionResult<Plane<T>, Line<T, 3>> : public CollisionResultBase
	{
	public:
		using Base      = CollisionResultBase;
		using PlaneType = Plane<T>;
		using LineType  = Line<T, 3>;

		static_assert(std::is_same_v<typename PlaneType::HVectorType, typename LineType::HVectorType>);

		using HVectorType = typename PlaneType::HVectorType;

		CollisionResult(bool collide) :
			Base(collide)
		{}

		CollisionResult(bool collide, const HVectorType& intersection) :
			Base(collide),
			m_Intersection(intersection)
		{}

		const HVectorType& Result() const { return m_Intersection; }

	private:
		HVectorType m_Intersection;
	};
}
