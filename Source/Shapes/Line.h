#pragma once

#include <LCN_Math/Source/Matrix/MatrixBlock.h>
#include <LCN_Math/Source/Geometry/Vector.h>

namespace LCN
{
	template<typename T, size_t Dim>
	class Line
	{
	public:
		using ValType     = T;
		using HVectorType = HVectorND<T, Dim>;

		Line(const HVectorType& origin, const HVectorType& direction) :
			m_Origin(origin),
			m_Direction(direction)
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