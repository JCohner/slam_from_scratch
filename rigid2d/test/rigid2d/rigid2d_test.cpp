#include <gtest/gtest.h>
#include <ros/ros.h>
TEST(TestSuite1, testCase1)
{
    ASSERT_EQ(2, 3);
    // There are many other ASSERT (fatal) and EXPECT (non-fatal, tests continue)
    // macros in google test
    // When first setting up tests, I like to write one that fails,
    // to ensure it is being run at the appropriate time
}
