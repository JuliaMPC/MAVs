#include "ros/ros.h"
#include <gtest/gtest.h>


TEST(MAVsTester, basicTest){

  int a = 1;
  int b = 2;
  int c = a+b;
  std::cout << "\n\n\nThis comes form inside of one test case\n\n\n";

  EXPECT_EQ(c, 3);
}

int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
