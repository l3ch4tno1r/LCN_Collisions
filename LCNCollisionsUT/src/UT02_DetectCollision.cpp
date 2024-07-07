#include <gtest/gtest.h>

#include "Collisions/Algorithms/CollisionAlgorithms.h"

namespace LCN::Collisions::UnitTests
{
    TEST(DetectCollisions, AABB_VS_Point)
    {
        Shapes::AABB<int, 2> aabb(LCN::Vector2Di{ 0, 0 }, LCN::Vector2Di{ 3, 3 });
        
        Shapes::Point<int, 2> pt1{ -1, 1 };
        Shapes::Point<int, 2> pt2{  1, 2 };
        Shapes::Point<int, 2> pt3{  4, 3 };

        //ASSERT_TRUE(LCN::DetectCollision(aabb, pt1));
    }
}