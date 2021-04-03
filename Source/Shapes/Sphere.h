#pragma once

#include <LCN_Math/Source/Geometry/Geometry.h>

namespace LCN
{
	////////////////////////////
	//-- Sphere N dimension --//
	////////////////////////////
	template<typename ValType, size_t Dim>
	class SphereND
	{
	public:
		using RefType     = ValType&;
		using HVectorType = HVectorND<ValType, Dim>;
		using RVectorType = VectorND<ValType, Dim>;

		SphereND(const RVectorType& center, ValType radius) :
			m_Center(center, ValType(1)),
			m_Radius(radius),
			m_SquareRadius(radius * radius)
		{}

		const HVectorType& Center() const { return m_Center; }
		      HVectorType& Center()       { return m_Center; }

		ValType Radius() const { return m_Radius; }

		void Radius(ValType radius);

		ValType SquareRadius() const { return m_SquareRadius; }

	private:
		HVectorType m_Center;

		ValType m_Radius;
		ValType m_SquareRadius;
	};

	template<typename ValType, size_t Dim>
	inline void SphereND<ValType, Dim>::Radius(ValType radius)
	{
		m_Radius       = radius;
		m_SquareRadius = radius * radius;
	}

	template<typename ValType>
	using Circle = SphereND<ValType, 2>;

	template<typename ValType>
	using Sphere = SphereND<ValType, 3>;

	using Circle2Df = Circle<float>;
	using Circle2Dd = Circle<double>;

	using Sphere3Df = Sphere<float>;
	using Sphere3Dd = Sphere<double>;
}