#include <gtest/gtest.h>

#include "Collisions/Shapes/Point.h"
#include "Collisions/Shapes/AABB.h"

namespace LCN::Collisions::UnitTests
{
    TEST(Shapes, AABB)
    {
        Shapes::AABB<int, 2> aabb{
            Shapes::Point<int, 2>{ 0, 0 },
            Shapes::Point<int, 2>{ 3, 3 },
        };
    }
}