#include <gtest/gtest.h>

#include "Collisions/Shapes/Point.h"
#include "Collisions/Shapes/AABB.h"

namespace LCN::Collisions::UnitTests
{
    TEST(Shapes, AABB2D)
    {
        Shapes::AABB<int, 2> aabb{
            Math::Vector2Di{ 1, 2 },
            Math::Vector2Di{ 3, 3 },
        };

        EXPECT_EQ(aabb.TopLeft(),     (Math::HVector2Di{ 1, 3, 1 }));
        EXPECT_EQ(aabb.TopRight(),    (Math::HVector2Di{ 3, 3, 1 }));
        EXPECT_EQ(aabb.BottomLeft(),  (Math::HVector2Di{ 1, 2, 1 }));
        EXPECT_EQ(aabb.BottomRight(), (Math::HVector2Di{ 3, 2, 1 }));

        EXPECT_EQ(aabb.Width(),  2);
        EXPECT_EQ(aabb.Height(), 1);
    }
}