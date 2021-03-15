#pragma once

#include <LCN_Math/Source/Geometry/Vector.h>

namespace LCN
{
	template<typename T, size_t Dim>
	class AABB
	{
	public:
		using ValType     = T;
		using HVectorType = HVectorND<T, Dim>;

		AABB(const HVectorType& topleft, ValType width, ValType height) :
			m_TopLeft(topleft),
			m_Width(width),
			m_Height(height)
		{}

	private:
		HVectorType m_TopLeft;

		ValType m_Width;
		ValType m_Height;
	};

	using AABB2Df = AABB<float, 2>;
}
