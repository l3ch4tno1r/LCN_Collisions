#pragma once

#include <cmath>

#include "LCN_Collisions/Source/Shapes/Point.h"
#include "LCN_Collisions/Source/Shapes/Line.h"
#include "LCN_Collisions/Source/Shapes/AABB.h"
#include "LCN_Collisions/Source/Shapes/Plane.h"
#include "LCN_Collisions/Source/Shapes/Hyperplane.h"
#include "LCN_Collisions/Source/Shapes/Sphere.h"

#include "LCN_Collisions/Source/Collisions/CollisionResult.h"

#define FUZZ_FACTOR T(0.001);

namespace LCN
{
#pragma region Dectection

	//////////////////////////////////
	//-- Collision detection only --//
	//////////////////////////////////

	// Allows symetry (DetectCollision(a, b) <=> DetectCollision(b, a))
	template<class Shape1, class Shape2>
	inline bool DetectCollision(const Shape1& shape1, const Shape2& shape2)
	{
		return DetectCollision(shape2, shape1);
	}

	// AABB vs point
	template<typename T>
	inline bool DetectCollision(const AABB<T, 2>& aabb, const Point<T, 2>& point)
	{
		const Point<T, 2>& topLeft = aabb.TopLeft();

		return
			(point.x() >= topLeft.x() && point.x() <= (topLeft.x() + aabb.Width())) &&
			(point.y() <= topLeft.y() && point.y() >= (topLeft.y() - aabb.Height()));
	}

	// AABB vs AABB
	template<typename T, size_t Dim>
	inline bool DetectCollision(const AABB<T, Dim>& aabb1, const AABB<T, Dim>& aabb2)
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
	inline bool DetectCollision(const Hyperplane<T, Dim>& hplane, const Line<T, Dim>& line)
	{
		return std::abs(hplane.Normal() | line.Direction()) > FUZZ_FACTOR;
	}

	// Plane vs Line
	template<typename T>
	inline bool DetectCollision(const Plane<T>& plane, const Line<T, 3>& line)
	{
		return std::abs(plane.Normal() | line.Direction()) > FUZZ_FACTOR;
	}

	// Plane vs Plane
	template<typename T>
	inline bool DetectCollision(const Plane<T>& plane1, const Plane<T>& plane2)
	{
		return std::abs(plane1.Normal() | plane2.Normal()) > FUZZ_FACTOR;
	}

	// Sphere vs Line
	template<typename T, size_t Dim>
	inline bool DetectCollision(const SphereND<T, Dim>& sphere, const Line<T, Dim>& line)
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
	template<class Shape1, class Shape2, class ResultType>
	inline void ComputeCollision(const Shape1& shape1, const Shape2& shape2, ResultType& result)
	{
		ComputeCollision(shape2, shape1, result);
	}

	// Hyperplane vs Line intersection
	template<typename T, size_t Dim>
	inline void ComputeCollision(const Hyperplane<T, Dim>& hplane, const Line<T, Dim>& line, HyperplaneVSLine<T, Dim>& result)
	{
		using HVectorType = typename Line<T, Dim>::HVectorType;

		if (!(result.m_Collide = DetectCollision(hplane, line)))
			return;

		const HVectorType& p = hplane.Origin();
		const HVectorType& n = hplane.Normal();
		const HVectorType& o = line.Origin();
		const HVectorType& d = line.Direction();

		HVectorType po = o - p;

		T k = -(po | n) / (d | n);
		
		result.m_Coordinate   = k;
		result.m_Intersection = k * d + o;
	}

	// Plane vs Line intersection
	template<typename T>
	inline void ComputeCollision(const Plane<T>& plane, const Line<T, 3>& line, PlaneVSLine<T>& result)
	{
		using HVectorType = typename Line<T, 3>::HVectorType;

		if (!(result.m_Collide = DetectCollision(plane, line)))
			return;

		const HVectorType& p = plane.Origin();
		const HVectorType& n = plane.Normal();
		const HVectorType& o = line.Origin();
		const HVectorType& d = line.Direction();

		HVector3Df po = o - p;

		T k = -(po | n) / (d | n);
		
		result.m_Coordinate   = k;
		result.m_Intersection = k * d + o;
	}

	// SphereND vs Line intersection
	template<typename T, size_t Dim>
	inline void ComputeCollision(const SphereND<T, Dim>& sphere, const Line<T, Dim>& line, SphereVSLine<T, Dim>& result)
	{
		using HVectorType = typename Line<T, Dim>::HVectorType;

		HVectorType co = line.Origin() - sphere.Center();
		const HVectorType& v = line.Direction();

		T r_2    = sphere.SquareRadius();
		T vDotCO = (v | co);
		T dCO_2  = co.SquareNorm();
		T delta  = 4 * (vDotCO * vDotCO - dCO_2 + r_2);

		if (!(result.m_Collide = (delta >= 0)))
			return;

		T t1 = (-2 * vDotCO - std::sqrt(delta)) / 2;
		T t2 = (-2 * vDotCO + std::sqrt(delta)) / 2;

		result.m_Intersections[0].Distance = t1;
		result.m_Intersections[1].Distance = t2;

		result.m_Intersections[0].Point = t1 * line.Direction() + line.Origin();
		result.m_Intersections[1].Point = t2 * line.Direction() + line.Origin();
	}

	// AABB vs AABB
	template<typename T, size_t Dim>
	inline void ComputeCollision(const AABB<T, Dim>& aabb1, const AABB<T, Dim>& aabb2, AABBVSAABB<T, Dim>& result)
	{
		using HVectorType = typename AABB<T, Dim>::HVectorType;

		HVectorType& maxmin = result.m_Intersection.Min();
		HVectorType& minmax = result.m_Intersection.Max();

		for (size_t i = 0; i < Dim; ++i)
		{
			maxmin[i] = std::max(aabb1.Min()[i], aabb2.Min()[i]);
			minmax[i] = std::min(aabb1.Max()[i], aabb2.Max()[i]);

			if (maxmin[i] < minmax[i])
				continue;

			result.m_Collide = false;

			return;
		}

		result.m_Collide = true;
	}

	// AABB vs Line
	template<typename T, size_t Dim>
	inline void ComputeCollision(const AABB<T, Dim>& aabb, const Line<T, Dim>& line, AABBVSLine<T, Dim>& result)
	{
		using ValType              = T;
		using RVectorType          = typename AABB<T, Dim>::RVectorType;
		using HyperplaneType       = Hyperplane<T, Dim>;
		using HyperplaneVSLineType = HyperplaneVSLine<T, Dim>;

		for (size_t face = 0; face < 2 * Dim; ++face)
		{
			RVectorType origin = aabb.Min().Vector();
			RVectorType normal;
			
			for (size_t i = 0; i < Dim; ++i)
				normal[i] = (i == face % Dim ? (face < Dim ? ValType(1) : ValType(-1)) : ValType(0));
			
			HyperplaneType hplane{ origin, normal };
			
			HyperplaneVSLineType tempResult;
			
			ComputeCollision(hplane, line, tempResult);

			if (!tempResult)
				continue;
		}
	}

#pragma endregion
}