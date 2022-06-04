#pragma once

#include <type_traits>
#include <array>

#include "LCN_Collisions/Source/Shapes/Plane.h"
#include "LCN_Collisions/Source/Shapes/Line.h"

namespace LCN
{
	/////////////////////////////////////////////
	//-- CollisionResult and specializations --//
	/////////////////////////////////////////////

	template<class Shape1, class Shape2>
	class CollisionResult;

#pragma region Hyperplane vs Line

	////////////////////////////
	//-- Hyperplane vs Line --//
	////////////////////////////

	template<typename T, size_t Dim>
	class CollisionResult<Hyperplane<T, Dim>, Line<T, Dim>>
	{
	public:
		using ValType   = T;
		using PlaneType = Hyperplane<T, Dim>;
		using LineType  = Line<ValType, Dim>;

		static_assert(std::is_same_v<typename PlaneType::HVectorType, typename LineType::HVectorType>);

		using HVectorType = typename PlaneType::HVectorType;

		CollisionResult() = default;

		CollisionResult(const HVectorType& inter, ValType coordinate) :
			m_Intersection{ inter },
			m_Coordinate{ coordinate }
		{}

		const HVectorType& Result() const { return m_Intersection; }

		const ValType Coordinate() const { return m_Coordinate; }

		template<typename T, size_t Dim>
		friend
		std::optional<CollisionResult<Hyperplane<T, Dim>, Line<T, Dim>>>
		ComputeCollision(
			const Hyperplane<T, Dim>& hplane,
			const Line<T, Dim>&       line);

	private:
		HVectorType m_Intersection;
		ValType     m_Coordinate;
	};

	template<typename T, size_t Dim>
	using HyperplaneVSLine = CollisionResult<Hyperplane<T, Dim>, Line<T, Dim>>;

#pragma endregion

#pragma region Plane vs Line

	/////////////////////////////////
	//-- Plane vs Line (3D only) --//
	/////////////////////////////////

	template<typename T>
	class CollisionResult<Plane<T>, Line<T, 3>>
	{
	public:
		using ValType   = T;
		using PlaneType = Plane<ValType>;
		using LineType  = Line<ValType, 3>;

		static_assert(std::is_same_v<typename PlaneType::HVectorType, typename LineType::HVectorType>);

		using HVectorType = typename PlaneType::HVectorType;

		const HVectorType& Result() const { return m_Intersection; }

		const ValType Coordinate() const { return m_Coordinate; }

	private:
		HVectorType m_Intersection;
		ValType     m_Coordinate;
	};

	template<typename T>
	using PlaneVSLine = CollisionResult<Plane<T>, Line<T, 3>>;

#pragma endregion

#pragma region SphereND vs Line

	//////////////////////////
	//-- SphereND vs Line --//
	//////////////////////////

	template<typename T, size_t Dim>
	class CollisionResult<SphereND<T, Dim>, Line<T, Dim>>
	{
	public:
		using ValType    = T;
		using SphereType = SphereND<T, Dim>;
		using LineType   = Line<T, Dim>;

		static_assert(std::is_same_v<typename SphereType::HVectorType, typename LineType::HVectorType>);

		using HVectorType = typename SphereType::HVectorType;

		struct IntersectionType
		{
			HVectorType Point;
			ValType     Distance;
		};

		using ConstIterator = typename std::array<IntersectionType, 2>::const_iterator;

		CollisionResult() :
			m_Intersections{ IntersectionType(), IntersectionType() }
		{}

		CollisionResult(
			const HVectorType& point1, ValType distance1,
			const HVectorType& point2, ValType distance2)
			: m_Intersections{ IntersectionType{ point1, distance1 }, IntersectionType{ point2, distance2 } }
		{}

		const IntersectionType& operator[](size_t i) { return m_Intersections[i]; }

		ConstIterator begin() const { return m_Intersections.begin(); }
		ConstIterator end()   const { return m_Intersections.end(); }

	private:
		std::array<IntersectionType, 2> m_Intersections;
	};

	template<typename T, size_t Dim>
	using SphereVSLine = CollisionResult<SphereND<T, Dim>, Line<T, Dim>>;

#pragma endregion

#pragma region AABB vs AABB

	//////////////////////
	//-- AABB vs AABB --//
	//////////////////////

	template<typename T, size_t Dim>
	class CollisionResult<AABB<T, Dim>, AABB<T, Dim>>
	{
	public:
		using AABBType    = AABB<T, Dim>;
		using HVectorType = typename AABBType::HVectorType;
		using RVectorType = typename AABBType::RVectorType;

		CollisionResult() = default;

		CollisionResult(const RVectorType& min, const RVectorType& max)
			: m_Intersection{ min, max }
		{}

		const AABBType& Result() const { return m_Intersection; }

	private:
		AABBType m_Intersection;
	};

	template<typename T, size_t Dim>
	using AABBVSAABB = CollisionResult<AABB<T, Dim>, AABB<T, Dim>>;

#pragma endregion

#pragma region AABB vs Line

	//////////////////////
	//-- AABB vs Line --//
	//////////////////////

	template<typename T, size_t Dim>
	class CollisionResult<AABB<T, Dim>, Line<T, Dim>>
	{
	public:
		using ValType  = T;
		using AABBType = AABB<ValType, Dim>;
		using LineType = Line<ValType, Dim>;

		static_assert(std::is_same_v<typename AABBType::HVectorType, typename LineType::HVectorType>);

		using HVectorType = typename AABBType::HVectorType;
		
		struct IntersectionType
		{
			size_t      FaceId;
			HVectorType Point;
			ValType     Distance;
		};

		using ConstIterator = typename std::array<IntersectionType, 2>::const_iterator;

	public:
		CollisionResult() :
			m_Intersections{ IntersectionType(), IntersectionType() }
		{}

		CollisionResult(const CollisionResult& other) :
			m_Intersections{ other.m_Intersections[0], other.m_Intersections[1] }
		{};

		const IntersectionType& operator[](size_t i) const { return m_Intersections[i]; }

		ConstIterator begin() const { return m_Intersections.begin(); }
		ConstIterator end()   const { return m_Intersections.end(); }

		template<typename T, size_t Dim>
		friend
		std::optional<CollisionResult<AABB<T, Dim>, Line<T, Dim>>>
		ComputeCollision(
			const AABB<T, Dim>&,
			const Line<T, Dim>&);

	private:
		std::array<IntersectionType, 2> m_Intersections;
	};

	template<typename T, size_t Dim>
	using AABBVSLine = CollisionResult<AABB<T, Dim>, Line<T, Dim>>;

#pragma endregion

	///////////////////////////////
	//-- Convenience utilities --//
	///////////////////////////////

	using HyperplaneVSLine2Df = HyperplaneVSLine<float, 2>;
	using HyperplaneVSLine3Df = HyperplaneVSLine<float, 3>;

	using PlaneVSLine3Df = PlaneVSLine<float>;

	using CircleVSLine2Df = SphereVSLine<float, 2>;
	using SphereVSLine3Df = SphereVSLine<float, 3>;

	using AABBVSAABB2Df = AABBVSAABB<float, 2>;
	using AABBVSAABB3Df = AABBVSAABB<float, 3>;

	using AABBVSLine2Df = AABBVSLine<float, 2>;
	using AABBVSLine3Df = AABBVSLine<float, 3>;
}
