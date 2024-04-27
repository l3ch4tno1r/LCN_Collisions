#pragma once

#include <optional>
#include <cmath>

#include "LCN_Collisions/Source/Shapes/Point.h"
#include "LCN_Collisions/Source/Shapes/Line.h"
#include "LCN_Collisions/Source/Shapes/AABB.h"
#include "LCN_Collisions/Source/Shapes/Plane.h"
#include "LCN_Collisions/Source/Shapes/Hyperplane.h"
#include "LCN_Collisions/Source/Shapes/Sphere.h"

#include "LCN_Collisions/Source/Collisions/CollisionResult.h"

#define FUZZ_FACTOR T(0.001)

namespace LCN
{
#pragma region Dectection

	//////////////////////////////////
	//-- Collision detection only --//
	//////////////////////////////////

	// Allows symetry (DetectCollision(a, b) <=> DetectCollision(b, a))
	template<class Shape1, class Shape2>
	inline bool
	DetectCollision(
		const Shape1& shape1,
		const Shape2& shape2)
	{
		return DetectCollision(shape2, shape1);
	}

	// AABB vs point
	template<typename T>
	inline bool
	DetectCollision(
		const AABB<T, 2>& aabb,
		const Point<T, 2>& point)
	{
		const Point<T, 2>& topLeft = aabb.TopLeft();

		return
			(point.x() >= topLeft.x() && point.x() <= (topLeft.x() + aabb.Width())) &&
			(point.y() <= topLeft.y() && point.y() >= (topLeft.y() - aabb.Height()));
	}

	// AABB vs AABB
	template<typename T, size_t Dim>
	inline bool
	DetectCollision(
		const AABB<T, Dim>& aabb1,
		const AABB<T, Dim>& aabb2)
	{
		using HVectorType = typename AABB<T, Dim>::HVectorType;

		for (size_t i = 0; i < Dim; ++i)
		{
			T maxmin = std::max(aabb1.Min()[i], aabb2.Min()[i]);
			T minmax = std::min(aabb1.Max()[i], aabb2.Max()[i]);

			if (maxmin > minmax)
				return false;
		}

		return true;
	}

	// Hyperplane vs Line
	template<typename T, size_t Dim>
	inline bool
	DetectCollision(
		const Hyperplane<T, Dim>& hplane,
		const Line<T, Dim>& line)
	{
		return std::abs(hplane.Normal() | line.Direction()) > FUZZ_FACTOR;
	}

	// Plane vs Line
	template<typename T>
	inline bool
	DetectCollision(
		const Plane<T>& plane,
		const Line<T, 3>& line)
	{
		return std::abs(plane.Normal() | line.Direction()) > FUZZ_FACTOR;
	}

	// Plane vs Plane
	template<typename T>
	inline bool
	DetectCollision(
		const Plane<T>& plane1,
		const Plane<T>& plane2)
	{
		return std::abs(plane1.Normal() | plane2.Normal()) > FUZZ_FACTOR;
	}

	// Sphere vs Line
	template<typename T, size_t Dim>
	inline bool
	DetectCollision(
		const SphereND<T, Dim>& sphere,
		const Line<T, Dim>& line)
	{
		using HVectorType = typename SphereND<T, Dim>::HVectorType;

		HVectorType oc = line.Origin() - sphere.Center();

		T ocDotDir = line.Direction() | oc;

		T squareDistance = oc.SquareNorm() - ocDotDir * ocDotDir;

		return squareDistance <= sphere.SquareRadius();
	}

#pragma endregion

#pragma region Computation

	///////////////////////////////////////
	//-- Compute collision information --//
	///////////////////////////////////////

	// Allows symetry (ComputeCollision(a, b) <=> DetectCollision(b, a))
	template<class Shape1, class Shape2>
	inline auto
	ComputeCollision(
		const Shape1& shape1,
		const Shape2& shape2)
	{
		return ComputeCollision(shape2, shape1);
	}

	///////////////////////////////
	//-- Collision computation --//
	///////////////////////////////

	// Hyperplane vs Line intersection
	template<typename T, size_t Dim>	
	std::optional<HyperplaneVSLine<T, Dim>>
	ComputeCollision(
		const Hyperplane<T, Dim>& hplane,
		const Line<T, Dim>& line)
	{
		using HVectorType = typename Line<T, Dim>::HVectorType;
		using ResultType  = std::optional<HyperplaneVSLine<T, Dim>>;

		if (!DetectCollision(hplane, line))
			return ResultType{ std::nullopt };

		const HVectorType& p = hplane.Origin();
		const HVectorType& n = hplane.Normal();
		const HVectorType& o = line.Origin();
		const HVectorType& d = line.Direction();

		HVectorType po = o - p;

		T k = -(po | n) / (d | n);

		return ResultType{ std::in_place, k * d + o, k };
	}

	// SphereND vs Line intersection
	template<typename T, size_t Dim>
	std::optional<SphereVSLine<T, Dim>>
	ComputeCollision(
		const SphereND<T, Dim>& sphere,
		const Line<T, Dim>& line)
	{
		using HVectorType = typename Line<T, Dim>::HVectorType;
		using ResultType  = std::optional<SphereVSLine<T, Dim>>;

		HVectorType co = line.Origin() - sphere.Center();
		const HVectorType& v = line.Direction();

		T r_2    = sphere.SquareRadius();
		T vDotCO = (v | co);
		T dCO_2  = co.SquareNorm();
		T delta  = 4 * (vDotCO * vDotCO - dCO_2 + r_2);

		if (delta < 0)
			return ResultType{ std::nullopt };

		T t1 = (-2 * vDotCO - std::sqrt(delta)) / 2;
		T t2 = (-2 * vDotCO + std::sqrt(delta)) / 2;

		return ResultType{
			std::in_place,
			t1 * line.Direction() + line.Origin(), t1,
			t2 * line.Direction() + line.Origin(), t2
		};
	}

	// AABB vs AABB
	template<typename T, size_t Dim>
	std::optional<AABBVSAABB<T, Dim>>
	ComputeCollision(
		const AABB<T, Dim>& aabb1,
		const AABB<T, Dim>& aabb2)
	{
		using RVectorType = typename AABB<T, Dim>::RVectorType;
		using ResultType  = std::optional<AABBVSAABB<T, Dim>>;

		RVectorType maxmin, minmax;

		for (size_t i = 0; i < Dim; ++i)
		{
			maxmin[i] = std::max(aabb1.Min()[i], aabb2.Min()[i]);
			minmax[i] = std::min(aabb1.Max()[i], aabb2.Max()[i]);

			if (maxmin[i] >= minmax[i])
				return ResultType{ std::nullopt };
		}

		return ResultType{ std::in_place, maxmin, minmax };
	}

	// AABB vs Line
	template<typename T, size_t Dim>
	std::optional<AABBVSLine<T, Dim>>
	ComputeCollision(
		const AABB<T, Dim>& aabb,
		const Line<T, Dim>& line)
	{
		using ResultType = std::optional<AABBVSLine<T, Dim>>;

		const auto& origin    = line.Origin();
		const auto& direction = line.Direction();
		const auto& min       = aabb.Min();
		const auto& max       = aabb.Max();

		T tmaxmin = -std::numeric_limits<T>::infinity();
		T tminmax =  std::numeric_limits<T>::infinity();

		size_t faceId0 = 0;
		size_t faceId1 = 2 * Dim - 1;

		AABBVSLine<T, Dim> result;

		for (size_t i = 0; i < Dim; ++i)
		{
			//if (std::abs(direction[i]) < FUZZ_FACTOR && min[i] <= origin[i] && origin[i] <= max[i])
			//	continue;

			T t1 = (min[i] - origin[i]) / direction[i];
			T t2 = (max[i] - origin[i]) / direction[i];

			T tmin = std::min(t1, t2);
			T tmax = std::max(t1, t2);

			T oldtmaxmin = tmaxmin;
			T oldtminmax = tminmax;

			tmaxmin = std::max(tmaxmin, tmin);
			tminmax = std::min(tminmax, tmax);

			if (oldtmaxmin != tmaxmin)
				result.m_Intersections[0].FaceId = t1 < t2 ? faceId0 : faceId1;

			if (oldtminmax != tminmax)
				result.m_Intersections[1].FaceId = t1 < t2 ? faceId1 : faceId0;

			++faceId0;
			--faceId1;
		}

		if (tmaxmin >= tminmax)
			return ResultType{ std::nullopt };

		result.m_Intersections[0].Distance = tmaxmin;
		result.m_Intersections[1].Distance = tminmax;

		result.m_Intersections[0].Point = tmaxmin * direction + origin;
		result.m_Intersections[1].Point = tminmax * direction + origin;

		return ResultType{ std::in_place, result };
	}

#pragma endregion
}