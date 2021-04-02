#pragma once

#include <type_traits>

#include "LCN_Collisions/Source/Shapes/Plane.h"
#include "LCN_Collisions/Source/Shapes/Line.h"

namespace LCN
{
	/////////////////////////////
	//-- CollisionResultBase --//
	/////////////////////////////

	class CollisionResultBase
	{
	public:
		inline operator bool() const { return m_Collide; }

	protected:
		bool m_Collide = false;
	};

	/////////////////////////////////////////////
	//-- CollisionResult and specializations --//
	/////////////////////////////////////////////

	template<class Shape1, class Shape2>
	class CollisionResult;

	template<typename T>
	class CollisionResult<Plane<T>, Line<T, 3>> : public CollisionResultBase
	{
	public:
		using ThisType  = CollisionResult<Plane<T>, Line<T, 3>>;
		using Base      = CollisionResultBase;
		using PlaneType = Plane<T>;
		using LineType  = Line<T, 3>;

		static_assert(std::is_same_v<typename PlaneType::HVectorType, typename LineType::HVectorType>);

		using HVectorType = typename PlaneType::HVectorType;

		const HVectorType& Result() const { return m_Intersection; }

		template<typename T>
		friend void ComputeCollision(const Plane<T>& plane, const Line<T, 3>& line, CollisionResult<Plane<T>, Line<T, 3>>& result);

	private:
		HVectorType m_Intersection;
	};

	template<typename T>
	using PlaneVSLineCollision = CollisionResult<Plane<T>, Line<T, 3>>;

	///////////////////////////////
	//-- Convenience utilities --//
	///////////////////////////////

	using PlaneVSLineCollision3f = PlaneVSLineCollision<float>;
}
