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

	template<CollisionPolicy Policy, class Shape1, class Shape2>
	CollisionResult<Shape1, Shape2> Collision(const Shape1& s1, const Shape2& s2);

	template<CollisionPolicy Policy, class Shape1, class Shape2>
	CollisionResult<Shape1, Shape2> Collision<CollisionPolicy::DetectionOnly>(const Shape1& s1, const Shape2& s2)
	{
		return CollisionResult<Shape1, Shape2>(DetectCollision(s1, s2));
	}
}