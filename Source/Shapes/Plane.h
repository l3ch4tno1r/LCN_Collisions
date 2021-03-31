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
			m_Origin(origin),
			m_Normal(normal)
		{}

		const HVectorType& Origin() const { return m_Origin; }
		      HVectorType& Origin()       { return m_Origin; }

		const HVectorType& Normal() const { return m_Normal; }
		      HVectorType& Normal()       { return m_Normal; }

	private:
		HVectorType m_Origin;
		HVectorType m_Normal;
	};
}