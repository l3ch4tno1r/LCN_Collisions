#pragma once

/*
#include <LCN_Math/Source/Matrix/MatrixBlock.h>
#include <LCN_Math/Source/Geometry/Vector.h>
*/

#include <LCN_Math/Source/Geometry/Geometry.h>

namespace LCN
{
	template<typename T, size_t Dim>
	class Line
	{
	public:
		using ValType     = T;
		using HVectorType = HVectorND<T, Dim>;
		using RVectorType = VectorND<T, Dim>;

		Line(const RVectorType& origin, const RVectorType& direction) :
			m_Origin(origin, ValType(1)),
			m_Direction(direction, ValType(0))
		{
			auto dirVector = m_Direction.Vector();

			ValType norm = m_Direction.Norm();
			dirVector = dirVector / norm;
		}

		      HVectorType& Origin()       { return m_Origin; }
		const HVectorType& Origin() const { return m_Origin; }

		      HVectorType& Direction()       { return m_Direction; }
		const HVectorType& Direction() const { return m_Direction; }

	private:
		HVectorType m_Origin;
		HVectorType m_Direction;
	};

	using Line2Df = Line<float, 2>;
	using Line3Df = Line<float, 3>;
}