#pragma once

#include "CollisionAlgorithms.h"
#include "CollisionResult.h"

namespace LCN
{
	enum class CollisionPolicy
	{
		DetectionOnly,
		ContactComputation
	};

	/////////////////////////
	//-- Collision class --//
	/////////////////////////

	template<CollisionPolicy Policy>
	class Collision
	{
	public:
		template<class Shape1, class Shape2>
		CollisionResult<Shape1, Shape2> operator()(const Shape1& s1, const Shape2& s2);
	};

	////////////////////////
	//-- Implementation --//
	////////////////////////

	template<CollisionPolicy Policy>
	template<class Shape1, class Shape2>
	inline CollisionResult<Shape1, Shape2> Collision<Policy>::operator()(const Shape1& s1, const Shape2& s2)
	{
		static_assert(false);
	}

	template<>
	template<class Shape1, class Shape2>
	inline CollisionResult<Shape1, Shape2> Collision<CollisionPolicy::DetectionOnly>::operator()(const Shape1& s1, const Shape2& s2)
	{
		return CollisionResult<Shape1, Shape2>(DetectCollision(s1, s2));
	}

	///////////////////
	//-- Utilities --//
	///////////////////

	using CollisionDetect  = Collision<CollisionPolicy::DetectionOnly>;
	using CollisionCompute = Collision<CollisionPolicy::ContactComputation>;
}