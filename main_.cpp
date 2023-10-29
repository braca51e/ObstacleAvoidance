#include "FreeSpacePolygon.h"

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

int main(int argc, char** argv) {

    // First argument is the path to the image
    if (argc != 2) {
        std::cout << "Usage: ./main <path_to_image>" << std::endl;
        return 1;
    }
    cv::Mat map = cv::imread(argv[1]); // Replace "your_image.jpg" with the path to your image file

    Point po(45, 70);  // Example origin point (x, y)
    double theta_o = 0;  // Example initial angle

    FreeSpacePolygon fsp(map);
    std::vector<Point> freeSpacePolygon = fsp.calculateFreeSpacePolygon(po, theta_o, map);
    // Draw points in the image
    cv::circle(map, cv::Point(po.y, po.x), 1, cv::Scalar(0, 255, 0), -1);
    for (int i = 0; i < freeSpacePolygon.size(); i++) {
        cv::circle(map, cv::Point(freeSpacePolygon[i].y, freeSpacePolygon[i].x), 1, cv::Scalar(0, 0, 255), -1);
    }
    cv::imshow("Free Space Polygon", map);
    cv::waitKey(0);

    return 0;
}
