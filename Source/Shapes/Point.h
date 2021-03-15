#pragma once

#include <LCN_Math/Source/Geometry/Vector.h>

namespace LCN
{
	template<typename T, size_t Dim>
	using Point = HVectorND<T, Dim>;

	using Point2Df = Point<float, 2>;
}