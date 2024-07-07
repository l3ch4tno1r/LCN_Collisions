#pragma once

/*
#include <LCN_Math/Source/Matrix/MatrixBlock.h>
#include <LCN_Math/Source/Geometry/Vector.h>
*/

#include <LCNMath/Geometry/Geometry.h>

namespace LCN::Collisions::Shapes
{
	template<typename T, size_t Dim>
	using Point = HVectorND<T, Dim>;

	using Point2Df = Point<float, 2>;
}