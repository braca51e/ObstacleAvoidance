//
// Created by Luis Bracamontes on 10/25/23.
//

#ifndef RRTSTAR_H
#define RRTSTAR_H

#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <opencv2/opencv.hpp>

#define MIN_NODE_DIST 5.05

struct Node {
    double x, y;
    int parent;
    double cost;
    Node(double x = 0.0, double y = 0.0, int parent = -1, double cost = 0.0) : x(x), y(y), parent(parent), cost(cost) {}
};

class RRTStarAR {
public:
    RRTStarAR(cv::Mat map, double startX, double startY, double goalX, double goalY, double stepSize, double maxIterations, double closeNodePenalty, double goalReachedRadius, double tau_obs)
            : map(map), startX(startX), startY(startY), goalX(goalX), goalY(goalY), stepSize(stepSize), maxIterations(maxIterations), closeNodePenalty(closeNodePenalty), goalReachedRadius(goalReachedRadius), tau_obs_(tau_obs) {
        // Initialize your RRT*AR here
    }

    void GenerateRRTStarAR();
    std::vector<std::vector<Node>> GenerateNPathsRRTStarAR(int N);
    std::vector<Node> GetPath();

private:
    cv::Mat map;
    double startX, startY, goalX, goalY;
    double stepSize;
    double maxIterations;
    double closeNodePenalty;
    double goalReachedRadius;
    double tau_obs_;

    std::vector<Node> nodes;

    Node GenerateRandomNode() const;
    int FindNearestNode(const Node& randomNode);
    Node Steer(const int& from, const Node& to) const;
    bool IsValidNode(const Node& node);
    std::pair<int, double> FindClosestAndCost(const std::vector<Node>& neighbors,  const Node& newNode, int nearest);
    void Rewire(const std::vector<Node>& neighbors, Node& newNode, int closestIndex);
    bool IsGoalReached(const Node& node) const;
    static std::vector<std::pair<int, int>> BresenhamLine(int x1, int y1, int x2, int y2);
    bool CollisionFree(const int& nearestNode, const Node& newNode);
    std::vector<Node> FindNeighbors(const Node& newNode);

};

#endif //RRTSTAR_H
