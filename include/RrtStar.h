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
    Node* parent;
    double cost;
    Node(double x = 0.0, double y = 0.0, Node* parent = nullptr, double cost = 0.0) : x(x), y(y), parent(parent), cost(cost) {}
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
    Node* FindNearestNode(const Node& randomNode);
    Node Steer(const Node* from, const Node& to) const;
    bool IsValidNode(const Node& node);
    std::pair<int, double> FindClosestAndCost(const Node& from, const Node& to);
    bool Rewire(Node& newNode, int closestIndex);
    bool IsGoalReached(const Node& node) const;
    static std::vector<std::pair<int, int>> BresenhamLine(int x1, int y1, int x2, int y2);
    bool CollisionFree(const Node* nearestNode, const Node& newNode);
    std::vector<Node> FindNeighboors(const Node& newNode);

};

#endif //RRTSTAR_H
