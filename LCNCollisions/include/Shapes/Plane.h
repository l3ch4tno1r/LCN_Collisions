#pragma once

#include <LCN_Math/Source/Geometry/Geometry.h>

namespace LCN
{
	template<typename T>
	class Plane
	{
	public:
		using ValType     = T;
		using HVectorType = HVector3Df;
		using RVectorType = Vector3Df;

		Plane(const RVectorType& origin, const RVectorType& normal) :
			m_Origin(origin, ValType(1)),
			m_Normal(normal, ValType(0))
		{
			auto normalVector = m_Normal.Vector();

			ValType norm = m_Normal.Norm();
			normalVector = normalVector / norm;
		}

		const HVectorType& Origin() const { return m_Origin; }
		      HVectorType& Origin()       { return m_Origin; }

		const HVectorType& Normal() const { return m_Normal; }
		      HVectorType& Normal()       { return m_Normal; }

	private:
		HVectorType m_Origin;
		HVectorType m_Normal;
	};

	using Plane3Df = Plane<float>;
}