#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

// Define grid parameters
const int gridSize = 50;
const double occupancyMax = 1.0;
const int npts = 10;  // Number of points to calculate
const int HCnpts = 8;  // Number of points to calculate
const double tau_obs = 0.8;  // Threshold for occupied cells
const double rC = 40;  // Example radius for the line
const double rHC = 6;  // Example radius for the half circle

// Define a structure to represent a point
struct Point {
    int x, y;

    Point(int x = 0.0, int y = 0.0) : x(x), y(y) {}
};

// Line equation given an angle and origin
std::vector<Point> line(double angle, Point origin, double radius) {
    std::vector<Point> points;
    for (double r = 1.0; r <= radius; r += 1.0) {
        int x = static_cast<int>(origin.x + r * cos((angle)));
        int y = static_cast<int>(origin.y + r * sin((angle)));
        points.push_back(Point(x, y));
    }
    return points;
}

// Half circle given an angle and origin
std::vector<Point> halfCircle(double angle, Point origin, double radius) {
    std::vector<Point> points;
    double start_angle = angle - M_PI/2;
    double end_angle = angle + M_PI/2;
    for (int i = 0; i < HCnpts; ++i) {
        double a = start_angle + (end_angle - start_angle) * i/HCnpts;
        int x = static_cast<int>(origin.x + radius * cos((a)));
        int y = static_cast<int>(origin.y + radius * sin((a)));
        points.push_back(Point(x, y));
    }
    return points;
}

// Half circle given an angle and origin
std::vector<Point> halfCircleShift(const std::vector<Point>& HCPts, int dx, int dy) {
    std::vector<Point> points;
    for (const Point& p : HCPts) {
        int x = static_cast<int>(p.x+dx);
        int y = static_cast<int>(p.y+dy);
        points.push_back(Point(x, y));
    }
    return points;
}

//Point findCollisionPoint(const std::vector<Point>& Lk, const std::vector<Point>& HCk, const cv::Mat& Pm) {
Point findCollisionPoint(const std::vector<Point>& Lk, std::vector<Point>& HCk, cv::Mat Pm) { // for visualization only
    Point collisionPoint;
    Point pprev = Lk[0];
    cv::Mat Pm_dummy = Pm.clone();

    bool found = false;
    // opencv are reversed
    for (const Point& p : Lk) {
            cv::circle(Pm_dummy, cv::Point(p.x, p.y), 1, cv::Scalar(255, 0, 0), -1);  // -1 means to fill the point
        }
    static int img_count = 0;
    for (const Point &p: Lk) {
        // deltas for shifting circle
        int dx = p.x - pprev.x;
        int dy = p.y - pprev.y;
        if (found) {
            break;
        }
        for (const Point &q: HCk) {
            int qx = static_cast<int>(q.x);
            int qy = static_cast<int>(q.y); // opencv are reversed for visualization only
            std::cout << qy << " " << qx << std::endl;
            cv::circle(Pm_dummy, cv::Point(q.x, q.y), 1, cv::Scalar(0, 255, 0), -1);
            cv::imshow("Point Image", Pm_dummy);
            cv::waitKey(1);
            //Probability of occupancy if grid == 0 not occupied if grid == 255 occupied
            double mean_pix_value = (Pm.at<cv::Vec3b>(qy, qx)[0] + Pm.at<cv::Vec3b>(qy, qx)[1] + Pm.at<cv::Vec3b>(qy, qx)[2])/3.0;
            // opencv are reversed
            if (qy >= 0 && qy < Pm.rows && qx >= 0 && qx < Pm.cols &&
                (mean_pix_value / 255.0) < tau_obs) {
                std::cout << qy << " " << qx << std::endl;
                std::cout << (Pm.at<cv::Vec3b>(qy, qx)) << std::endl;
                cv::circle(Pm_dummy, cv::Point(q.x, q.y), 2, cv::Scalar(0, 0, 255), -1);
                cv::imshow("Point Image", Pm_dummy);
                cv::waitKey(2);
                collisionPoint = q;  // collision cell
                found = true;
                // Save image for visualization
                img_count += 1;
                std::string image_name = "/home/luis/ObstacleAvoidance/tmp/image_" + std::to_string(img_count) + ".png";
                //cv::imwrite(image_name, Pm_dummy);
                //img_count = 0;
                break;
            }
            // Save image for visualization
            img_count += 1;
            std::string image_name = "/home/luis/ObstacleAvoidance/tmp/image_" + std::to_string(img_count) + ".png";
            //cv::imwrite(image_name, Pm_dummy);
        }

        HCk = halfCircleShift(HCk, dx, dy);  // shift HCk
        pprev = p;
    }
    if (!found){
        // last point in Lk and check boundaries
        collisionPoint = Lk.back();
        if (collisionPoint.y > Pm.rows)
            collisionPoint.y = Pm.rows;
        if (collisionPoint.x > Pm.cols)
            collisionPoint.x = Pm.cols;
    }
    return collisionPoint;
}

// Function to calculate the free-space polygon
std::vector<Point> calculateFreeSpacePolygon(Point po, double theta_o, const cv::Mat& Pm) {
    std::vector<Point> P;
    //@TODO convert grid element to distance in m
    std::cout << Pm.at<cv::Vec3b>(po.y, po.x)[0]/255.0 << std::endl; // opencv are reversed

    for (int k = 0; k < npts; ++k) {
        double theta_k = theta_o + M_PI - (2.0 * M_PI * k)/npts;
        std::vector<Point> Lk = line(theta_k, po, rC);
        std::vector<Point> HCk = halfCircle(theta_k, po, rHC);

        // Find the collision point
        cv::Mat Pm_draw = Pm.clone();
        Point collisionPoint = findCollisionPoint(Lk, HCk, Pm_draw);
        P.push_back(collisionPoint); // collision point
    }

    return P;
}

int main(int argc, char** argv) {
    // Load an image map from a file
    cv::Mat map = cv::imread("/home/luis/ObstacleAvoidance/maps/free-space-poly-white.pgm"); // Replace "your_image.jpg" with the path to your image file
    // The algorithm for free-space polygon construction can be implemented here
    // Define other algorithm parameters
    Point po(53, 45);  // Example origin point (x, y)
    double theta_o = 0;  // Example initial angle

    // Calculate the free-space polygon
    std::vector<Point> freeSpacePolygon = calculateFreeSpacePolygon(po, theta_o, map);

    // Print the resulting free-space polygon points
    // draw init point in map
    cv::circle(map, cv::Point(po.x, po.y), 2, cv::Scalar(255, 0, 0), -1);  // -1 means to fill the point
    for (const Point& p : freeSpacePolygon) {
        std::cout << "x: " << p.y << ", y: " << p.x << std::endl;
        cv::circle(map, cv::Point(p.x, p.y), 1, cv::Scalar(0, 0, 255), -1);  // -1 means to fill the point
    }

    // Draw a line connecting each point
    for (int i = 0; i < freeSpacePolygon.size() - 1; ++i) {
        cv::line(map, cv::Point(freeSpacePolygon[i].x, freeSpacePolygon[i].y), cv::Point(freeSpacePolygon[i + 1].x, freeSpacePolygon[i + 1].y), cv::Scalar(0, 255, 0), 1);
    }
    // connect last point
    cv::line(map, cv::Point(freeSpacePolygon.back().x, freeSpacePolygon.back().y), cv::Point(freeSpacePolygon[0].x, freeSpacePolygon[0].y), cv::Scalar(0, 255, 0), 1);

    cv::imshow("Point Image", map);
    cv::waitKey(0);
    std::string image_name = "/home/luis/ObstacleAvoidance/free-space-poly-white.png";
    //resize map to 220x220
    cv::resize(map, map, cv::Size(220, 220));
    cv::imwrite(image_name, map);

    return 0;
}
