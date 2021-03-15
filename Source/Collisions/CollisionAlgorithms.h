#pragma once

#include "LCN_Collisions/Source/Shapes/Point.h"
#include "LCN_Collisions/Source/Shapes/Line.h"
#include "LCN_Collisions/Source/Shapes/AABB.h"

namespace LCN
{
	// Allows to perform symetry (DetectCollision(a, b) <=> DetectCollision(b, a))
	template<class Shape1, class Shape2>
	inline bool DetectCollision(const Shape1& shape1, const Shape2& shape2)
	{
		return DetectCollision(shape2, shape1);
	}

	// AABB vs point
	template<typename T>
	bool DetectCollision(const AABB<T, 2>& aabb, const Point<T, 2>& point)
	{
		const Point<T, 2>& topLeft = aabb.TopLeft();

		return
			(point.x() >= topLeft.x() && point.x() <= (topLeft.x() + aabb.Width())) &&
			(point.y() <= topLeft.y() && point.y() >= (topLeft.y() - aabb.Height()));
	}

	// AABB vs AABB
	template<typename T>
	bool DetectCollision(const AABB<T, 2>& aabb1, const AABB<T, 2>& aabb2)
	{
		T TLx1 = aabb1.TopLeft().x(), TLx2 = aabb2.TopLeft().x();
		T TLy1 = aabb1.TopLeft().y(), TLy2 = aabb2.TopLeft().y();
		T BRx1 = aabb1.TopLeft().x() + aabb1.Width(),  BRx2 = aabb2.TopLeft().x() + aabb2.Width();
		T BRy1 = aabb1.TopLeft().y() - aabb1.Height(), BRy2 = aabb2.TopLeft().y() - aabb2.Height();

		T minx = std::max(TLx1, TLx2), maxx = std::min(BRx1, BRx2);
		T maxy = std::min(TLy1, TLy2), miny = std::max(BRy1, BRy2);

		return minx <= maxx && miny <= maxy;
	}
}