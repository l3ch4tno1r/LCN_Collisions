#pragma once

#include "CollisionResult.h"

#include "LCN_Collisions/Source/Shapes/Point.h"
#include "LCN_Collisions/Source/Shapes/Line.h"
#include "LCN_Collisions/Source/Shapes/AABB.h"

namespace LCN
{
	enum class CollisionPolicy
	{
		DetectionOnly,
		ContactComputation
	};

	template<CollisionPolicy Policy, class C1, class C2>
	CollisionResult<C1, C2> Collision(const C1& c1, const C2& c2);

	template<CollisionPolicy Policy, class C1, class C2>
	CollisionResult<C1, C2> Collision<CollisionPolicy::DetectionOnly>(const C1& c1, const C2& c2)
	{
		return CollisionResult<C1, C2>(DetectCollision(c1, c2));
	}

	template<class C1, class C2>
	auto& CollisionDetectionOnly = Collision<CollisionPolicy::DetectionOnly>;
}