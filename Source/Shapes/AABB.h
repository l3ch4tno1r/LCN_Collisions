#pragma once

#include <LCN_Math/Source/Geometry/Geometry.h>

#ifdef _DEBUG
#define DEBUG
#endif // _DEBUG

#include <Utilities/Source/ErrorHandling.h>

namespace LCN
{
	//////////////
	//-- AABB --//
	//////////////

	template<typename T, size_t Dim>
	class AABB
	{
	public:
		using ValType     = T;
		using HVectorType = HVectorND<ValType, Dim>;
		using RVectorType = VectorND<ValType, Dim>;

		enum
		{
			NumFaces = 2 * Dim
		};

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

	///////////////////////////////
	//-- AABB normals sequence --//
	///////////////////////////////

	template<class AABBType, size_t ... Args>
	struct AABBNormalsData
	{
		using HVectorType      = typename AABBType::HVectorType;
		using HVectorArrayType = std::array<HVectorType, sizeof...(Args)>;

		static const HVectorArrayType Normals;
	};

	template<class AABBType, size_t N, size_t ... Args>
	struct IdxGenerator
	{
		using DataType = typename IdxGenerator<AABBType, N - 1, N, Args...>::DataType;
	};

	template<class AABBType, size_t ... Args>
	struct IdxGenerator<AABBType, 0, Args...>
	{
		using DataType = typename AABBNormalsData<AABBType, Args...>;
	};

	template<class AABBType>
	struct AABBNormals
	{
		using HVectorType      = typename AABBType::HVectorType;
		using DataType         = typename IdxGenerator<AABBType, AABBType::NumFaces>::DataType;
		using HVectorArrayType = typename DataType::HVectorArrayType;

		inline static const HVectorArrayType& Normals() { return DataType::Normals; }

		static HVectorType GenerateNormals(size_t i)
		{
			size_t _i = i % HVectorType::Dim;

			HVectorType result(_i);

			if (i < HVectorType::Dim)
				result[_i] = HVectorType::ValType(-1);

			return result;
		}
	};

	template<class AABBType, size_t ... Args>
	const std::array<typename AABBType::HVectorType, sizeof...(Args)> AABBNormalsData<AABBType, Args...>::Normals = { AABBNormals<AABBType>::GenerateNormals(Args - 1)... };

	////////////////////////
	//-- Shortcut types --//
	////////////////////////

	using AABB2Df = AABB<float, 2>;
	using AABB3Df = AABB<float, 3>;

	using AABBNormals2Df = AABBNormals<AABB2Df>;
	using AABBNormals3Df = AABBNormals<AABB3Df>;
}
