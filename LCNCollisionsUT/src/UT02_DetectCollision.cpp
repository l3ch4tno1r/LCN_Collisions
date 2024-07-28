#include <gtest/gtest.h>

#include "Collisions/Algorithms/CollisionAlgorithms.h"

namespace LCN::Collisions::UnitTests
{
    TEST(DetectCollisions, AABB_VS_Point)
    {
        Shapes::AABB<int, 2> aabb{
            LCN::Math::Vector2Di{ 0, 0 },
            LCN::Math::Vector2Di{ 3, 3 }
        };
        
        Shapes::Point<int, 2> pt1{ -1, 1 };
        Shapes::Point<int, 2> pt2{  4, 3 };
        Shapes::Point<int, 2> pt3{  1, 2 };

        EXPECT_FALSE(Algorithms::DetectCollision(aabb, pt1));
        EXPECT_FALSE(Algorithms::DetectCollision(aabb, pt2));

        EXPECT_TRUE(Algorithms::DetectCollision(aabb, pt3));
    }
}