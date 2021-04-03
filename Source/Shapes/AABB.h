#pragma once

#include <LCN_Math/Source/Geometry/Geometry.h>

namespace LCN
{
	template<typename T, size_t Dim>
	class AABB;

	template<typename T>
	class AABB<T, 2>
	{
	public:
		using ValType     = T;
		using RefType     = ValType&;
		using HVectorType = HVectorND<T, 2>;
		using RVectorType = VectorND<T, 2>;

		AABB(const RVectorType& topleft, ValType width, ValType height) :
			m_TopLeft(topleft, ValType(1)),
			m_Width(width),
			m_Height(height)
		{}

		inline       HVectorType& TopLeft()       { return m_TopLeft; }
		inline const HVectorType& TopLeft() const { return m_TopLeft; }

		inline RefType Width()       { return m_Width; }
		inline ValType Width() const { return m_Width; }

		inline RefType Height()       { return m_Height; }
		inline ValType Height() const { return m_Height; }

	private:
		HVectorType m_TopLeft;

		ValType m_Width;
		ValType m_Height;
	};

	using AABB2Df = AABB<float, 2>;
}
