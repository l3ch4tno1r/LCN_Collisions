#pragma once

#include <type_traits>
#include <array>

#include "LCN_Collisions/Source/Shapes/Plane.h"
#include "LCN_Collisions/Source/Shapes/Line.h"

namespace LCN
{
	/////////////////////////////
	//-- CollisionResultBase --//
	/////////////////////////////

	class CollisionResultBase
	{
	public:
		inline operator bool() const { return m_Collide; }

	protected:
		bool m_Collide = false;
	};

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
	class CollisionResult<Hyperplane<T, Dim>, Line<T, Dim>> : public CollisionResultBase
	{
	public:
		using ValType = T;
		using PlaneType = Hyperplane<T, Dim>;
		using LineType  = Line<ValType, Dim>;

		static_assert(std::is_same_v<typename PlaneType::HVectorType, typename LineType::HVectorType>);

		using HVectorType = typename PlaneType::HVectorType;

		const HVectorType& Result() const { return m_Intersection; }

		const ValType Coordinate() const { return m_Coordinate; }

		template<typename T, size_t Dim>
		friend void ComputeCollision(const Hyperplane<T, Dim>& hplane, const Line<T, Dim>& line, CollisionResult<Hyperplane<T, Dim>, Line<T, Dim>>& result);

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
	class CollisionResult<Plane<T>, Line<T, 3>> : public CollisionResultBase
	{
	public:
		using ValType   = T;
		using PlaneType = Plane<ValType>;
		using LineType  = Line<ValType, 3>;

		static_assert(std::is_same_v<typename PlaneType::HVectorType, typename LineType::HVectorType>);

		using HVectorType = typename PlaneType::HVectorType;

		const HVectorType& Result() const { return m_Intersection; }

		const ValType Coordinate() const { return m_Coordinate; }

		template<typename T>
		friend void ComputeCollision(const Plane<T>& plane, const Line<T, 3>& line, CollisionResult<Plane<T>, Line<T, 3>>& result);

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
	class CollisionResult<SphereND<T, Dim>, Line<T, Dim>> : public CollisionResultBase
	{
	public:
		using ValType    = T;
		using Base       = CollisionResultBase;
		using SphereType = SphereND<T, Dim>;
		using LineType   = Line<T, Dim>;

		static_assert(std::is_same_v<typename SphereType::HVectorType, typename LineType::HVectorType>);

		using HVectorType = typename SphereType::HVectorType;

		struct IntesectionType
		{
			HVectorType Point;
			ValType     Distance;
		};

		using ConstIterator = typename std::array<IntesectionType, 2>::const_iterator;

		CollisionResult() :
			Base(),
			m_Intersections{ IntesectionType(), IntesectionType() }
		{}

		template<typename T, size_t Dim>
		friend void ComputeCollision(const SphereND<T, Dim>& sphere, const Line<T, Dim>& line, CollisionResult<SphereND<T, Dim>, Line<T, Dim>>& result);

		const IntesectionType& operator[](size_t i) { return m_Intersections[i]; }

		ConstIterator begin() const { return m_Intersections.begin(); }
		ConstIterator end()   const { return m_Intersections.end(); }

	private:
		std::array<IntesectionType, 2> m_Intersections;
	};

	template<typename T, size_t Dim>
	using SphereVSLine = CollisionResult<SphereND<T, Dim>, Line<T, Dim>>;

#pragma endregion

#pragma region AABB vs AABB

	//////////////////////
	//-- AABB vs AABB --//
	//////////////////////

	template<typename T, size_t Dim>
	class CollisionResult<AABB<T, Dim>, AABB<T, Dim>> : public CollisionResultBase
	{
	public:
		using AABBType    = AABB<T, Dim>;
		using HVectorType = typename AABBType::HVectorType;

		CollisionResult() :
			m_Intersection({{0, 0}, 0, 0})
		{}

		const AABBType& Result() const { return m_Intersection; }

		template<typename T, size_t Dim>
		friend void ComputeCollision(const AABB<T, Dim>& aabb1, const AABB<T, Dim>& aabb2, CollisionResult<AABB<T, Dim>, AABB<T, Dim>>& result);

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
	class CollisionResult<AABB<T, Dim>, Line<T, Dim>> : public CollisionResultBase
	{
	public:
		using AABBType = AABB<T, Dim>;
		using LineType = Line<T, Dim>;

		static_assert(std::is_same_v<typename AABBType::HVectorType, typename LineType::HVectorType>);

		using HVectorType = typename AABBType::HVectorType;
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
}
