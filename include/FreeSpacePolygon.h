//
// Created by Luis Bracamontes on 10/24/23.
//

#ifndef FREEPOLYGONSPACE_FREESPACEPOLYGON_H
#define FREEPOLYGONSPACE_FREESPACEPOLYGON_H

#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

// Define a structure to represent a point
struct Point {
    int x, y;
    Point(int x = 0.0, int y = 0.0) : x(x), y(y) {}
};

class FreeSpacePolygon {
public:
    FreeSpacePolygon(const cv::Mat map) : map_(map){}
    ~FreeSpacePolygon() {}
    std::vector<Point> calculateFreeSpacePolygon(Point po, double theta_o, const cv::Mat& Pm);

private:

    std::vector<Point> line(double angle, Point origin, double radius); // Line equation given an angle and origin
    std::vector<Point> halfCircle(double angle, Point origin, double radius); // Half circle given an angle and origin
    std::vector<Point> halfCircleShift(const std::vector<Point>& HCPts, int dx, int dy); // Half circle given an angle and origin
    std::vector<Point> halfCircleWithBresenham(double angle, Point origin, double radius);
    std::vector<Point> lineWithBresenham(double angle, Point origin, double radius);
    Point findCollisionPoint(const std::vector<Point>& Lk, std::vector<Point>& HCk, const cv::Mat& Pm);
    cv::Mat map_;
    const int npts_ = 10;  // Number of points to calculate
    const int HCnpts_ = 8;  // Number of points to calculate
    const double tau_obs_ = 0.8;  // Threshold for occupied cells
    const int rC_ = 40;  // Example radius for the line
    const int rHC_ = 6;  // Example radius for the half circle
};

#endif //FREEPOLYGONSPACE_FREESPACEPOLYGON_H
