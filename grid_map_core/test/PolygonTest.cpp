/*
 * GridMapTest.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: Martin Wermelinger, Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/Polygon.hpp"

// gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
using namespace grid_map;

TEST(checkConvexHull, createHull)
{
  grid_map::Polygon polygon1;
  polygon1.addVertex(Vector2d(0.0, 0.0));
  polygon1.addVertex(Vector2d(1.0, 1.0));
  polygon1.addVertex(Vector2d(0.0, 1.0));
  polygon1.addVertex(Vector2d(1.0, 0.0));

  grid_map::Polygon polygon2;
  polygon2.addVertex(Vector2d(0.5, 0.5));
  polygon2.addVertex(Vector2d(0.5, 1.5));
  polygon2.addVertex(Vector2d(1.5, 0.5));
  polygon2.addVertex(Vector2d(1.5, 1.5));

  grid_map::Polygon hull;
  hull = hull.convexHull(polygon1, polygon2);

  EXPECT_EQ(6, hull.nVertices());
  EXPECT_TRUE(hull.isInside(Vector2d(0.5, 0.5)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.01, 1.49)));
}

TEST(checkConvexHullCircles, createHull)
{
  Position center1(0.0, 0.0);
  Position center2(1.0, 0.0);
  double radius = 0.5;
  const int nVertices = 15;

  grid_map::Polygon hull;
  hull = hull.convexHullCircles(center1, center2, radius);
  EXPECT_EQ(20, hull.nVertices());
  EXPECT_TRUE(hull.isInside(Vector2d(-0.25, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.5, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.5, 0.4)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.5, 0.6)));
  EXPECT_FALSE(hull.isInside(Vector2d(1.5, 0.2)));

  hull = hull.convexHullCircles(center1, center2, radius, nVertices);
  EXPECT_EQ(nVertices + 1, hull.nVertices());
  EXPECT_TRUE(hull.isInside(Vector2d(-0.25, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.5, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.5, 0.4)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.5, 0.6)));
  EXPECT_FALSE(hull.isInside(Vector2d(1.5, 0.2)));
}

TEST(checkConvexHullCircle, createHull)
{
  Position center(0.0, 0.0);
  double radius = 0.5;
  const int nVertices = 15;

  grid_map::Polygon hull;
  hull = hull.convexHullCircle(center, radius);

  EXPECT_EQ(20, hull.nVertices());
  EXPECT_TRUE(hull.isInside(Vector2d(-0.25, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.49, 0.0)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.5, 0.4)));
  EXPECT_FALSE(hull.isInside(Vector2d(1.0, 0.0)));

  hull = hull.convexHullCircle(center, radius, nVertices);
  EXPECT_EQ(nVertices, hull.nVertices());
  EXPECT_TRUE(hull.isInside(Vector2d(-0.25, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.49, 0.0)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.5, 0.4)));
  EXPECT_FALSE(hull.isInside(Vector2d(1.0, 0.0)));
}

TEST(convertToInequalityConstraints, triangle1)
{
  grid_map::Polygon polygon({Position(1.0, 1.0), Position(0.0, 0.0), Position(1.1, -1.1)});
  MatrixXd A;
  VectorXd b;
  ASSERT_TRUE(polygon.convertToInequalityConstraints(A, b));
  EXPECT_NEAR(-1.3636, A(0, 0), 1e-4);
  EXPECT_NEAR( 1.3636, A(0, 1), 1e-4);
  EXPECT_NEAR(-1.5000, A(1, 0), 1e-4);
  EXPECT_NEAR(-1.5000, A(1, 1), 1e-4);
  EXPECT_NEAR( 2.8636, A(2, 0), 1e-4);
  EXPECT_NEAR( 0.1364, A(2, 1), 1e-4);
  EXPECT_NEAR( 0.0000, b(0), 1e-4);
  EXPECT_NEAR( 0.0000, b(1), 1e-4);
  EXPECT_NEAR( 3.0000, b(2), 1e-4);
}

TEST(convertToInequalityConstraints, triangle2)
{
  grid_map::Polygon polygon({Position(-1.0, 0.5), Position(-1.0, -0.5), Position(1.0, -0.5)});
  MatrixXd A;
  VectorXd b;
  ASSERT_TRUE(polygon.convertToInequalityConstraints(A, b));
  EXPECT_NEAR(-1.5000, A(0, 0), 1e-4);
  EXPECT_NEAR( 0.0000, A(0, 1), 1e-4);
  EXPECT_NEAR( 0.0000, A(1, 0), 1e-4);
  EXPECT_NEAR(-3.0000, A(1, 1), 1e-4);
  EXPECT_NEAR( 1.5000, A(2, 0), 1e-4);
  EXPECT_NEAR( 3.0000, A(2, 1), 1e-4);
  EXPECT_NEAR( 1.5000, b(0), 1e-4);
  EXPECT_NEAR( 1.5000, b(1), 1e-4);
  EXPECT_NEAR( 0.0000, b(2), 1e-4);
}
