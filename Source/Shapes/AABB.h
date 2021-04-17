#pragma once

#include <LCN_Math/Source/Geometry/Geometry.h>

#ifdef _DEBUG
#define DEBUG
#endif // _DEBUG

#include <Utilities/Source/ErrorHandling.h>

namespace LCN
{
	template<typename T, size_t Dim>
	class AABB
	{
	public:
		using ValType     = T;
		using HVectorType = HVectorND<ValType, Dim>;
		using RVectorType = VectorND<ValType, Dim>;

		AABB(const RVectorType& min, const RVectorType& max) :
			m_Min(min, ValType(1)),
			m_Max(max, ValType(1))
		{
			for (size_t i = 0; i < Dim; ++i)
				ASSERT(m_Min[i] <= m_Max[i]);
		}

		inline const HVectorType& Min() const { return m_Min; }
		inline       HVectorType& Min()       { return m_Min; }

		inline const HVectorType& Max() const { return m_Max; }
		inline       HVectorType& Max()       { return m_Max; }

		ValType Length() const
		{
			static_assert(Dim >= 3);

			return m_Max.x() - m_Min.x();
		}

		ValType Width() const
		{
			if constexpr (Dim == 2)
				return m_Max.x() - m_Min.x();
			else
				return m_Max.y() - m_Min.y();
		}

		ValType Height() const
		{
			if constexpr (Dim == 2)
				return m_Max.y() - m_Min.y();
			else
				return m_Max.z() - m_Min.z();
		}

	private:
		HVectorType m_Min;
		HVectorType m_Max;
	};

	using AABB2Df = AABB<float, 2>;
	using AABB3Df = AABB<float, 3>;
}
