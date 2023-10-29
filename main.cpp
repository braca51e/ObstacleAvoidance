#include "RrtStar.h"

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
    cv::Mat map = cv::imread(argv[1]); // Replace "your_image.jpg" with the path to your image file// Set your RRT*AR parameters

    double startX = 66.0;
    double startY = 45.0;
    double goalX = 35.0;
    double goalY = 70.0;
    double stepSize = 5.0;
    double maxIterations = 1000;
    double closeNodePenalty = 1.0; // Adjust this penalty as needed
    double goalReachedRadius = 3.0;
    double tau_obs = 0.8;

    RRTStarAR planner(map, startX, startY, goalX, goalY, stepSize, maxIterations, closeNodePenalty, goalReachedRadius, tau_obs);
    planner.GenerateRRTStarAR();
    std::vector<Node> path = planner.GetPath();

    // print each node in the path
    for (int i = 0; i < path.size(); i++) {
        std::cout << "Node " << i << ": " << path[i].x << ", " << path[i].y << std::endl;
    }

    return 0;
}
