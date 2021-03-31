#pragma once

#include <LCN_Math/Source/Matrix/MatrixBlock.h>
#include <LCN_Math/Source/Geometry/Vector.h>

namespace LCN
{
	template<typename T>
	class Plane
	{
	public:
		using ValType     = T;
		using HVectorType = HVector3Df;

		Plane(const HVectorType& origin, const HVectorType& normal) :
			Origin(origin),
			Normal(normal)
		{}

	private:
		HVectorType Origin;
		HVectorType Normal;
	};
}