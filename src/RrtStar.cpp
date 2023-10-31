//
// Created by Luis Bracamontes on 10/25/23.
//

#include "RrtStar.h"

void RRTStarAR::GenerateRRTStarAR() {

    // Initialize the tree with the starting node
    nodes.clear();
    nodes.push_back({startX, startY, nullptr, 0.0});

    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        Node randomNode = GenerateRandomNode();
        Node* nearestNode = FindNearestNode(randomNode);
        // Attempt to steer from the nearest node towards the random node
        Node newNode = Steer(nearestNode, randomNode);
        // Check if the new node is valid (not in collision)
        if (CollisionFree(nearestNode, newNode)) {
            //@TODO: Get all neighboors within a radius
            //std::vector<Node> neighboors = FindNeighboors(newNode);
            newNode.parent = &nodes.back();
            nodes.push_back(newNode);
        }

        if (IsGoalReached(newNode)){
            break;
        }
    }
}

// Function to generate a random node within the map bounds
Node RRTStarAR::GenerateRandomNode() const {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> xDist(0, map.rows - 1);
    std::uniform_real_distribution<double> yDist(0, map.cols - 1);

    double x = xDist(gen);
    double y = yDist(gen);

    return {x, y, nullptr, 0.0};
}

// Function to find the nearest node in the tree to the given random node
Node* RRTStarAR::FindNearestNode(const Node& randomNode) {
    // Assign max value
    double minDist = std::numeric_limits<double>::max();
    Node* nearestNode = nullptr;

    for (int i = 0; i < nodes.size(); ++i) {
        double dist = std::sqrt(std::pow(randomNode.x - nodes[i].x, 2) + std::pow(randomNode.y - nodes[i].y, 2));
        if (dist < minDist) {
            nearestNode = &nodes[i];
            minDist = dist;
        }
    }
    return nearestNode;
}

// Function to steer from the 'from' node towards the 'to' node with a specified step size
Node RRTStarAR::Steer(const Node* from, const Node& to) const {
    double dx = to.x - from->x;
    double dy = to.y - from->y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist <= stepSize) {
        return to;
    }

    double ratio = stepSize / dist;
    double newX = from->x + dx * ratio;
    double newY = from->y + dy * ratio;

    return {newX, newY, nullptr, 0.0};
}

// Function to check if a node is valid (not in collision with the map)
//bool RRTStarAR::IsValidNode(const Node& node) {
bool RRTStarAR::CollisionFree(const Node* nearestNode, const Node& newNode) {
    int x1 = static_cast<int>(nearestNode->x);
    int y1 = static_cast<int>(nearestNode->y);
    int x2 = static_cast<int>(newNode.x);
    int y2 = static_cast<int>(newNode.y);

    // Get the line segment between the nodes using Bresenham's line algorithm
    std::vector<std::pair<int, int>> line = BresenhamLine(x1, y1, x2, y2);

    // Check if any of the cells in the line are occupied
    for (const auto& point : line){
        int x = point.first;
        int y = point.second;

        // check if pixel is within bounds
        if (x < 0 || x >= map.rows || y < 0 || y >= map.cols) {
            return false;
        }

        // Check if the pixel is occupied
        double mean_pix_value = (map.at<cv::Vec3b>(x, y)[0] + map.at<cv::Vec3b>(x, y)[1] +
                                 map.at<cv::Vec3b>(x, y)[2])/3.0;
        mean_pix_value /= 255.0;
        if (mean_pix_value < tau_obs_) {
            return false;
        }
    }
    return true;
}

// Function to find the closest neighbor to the 'to' node and calculate the cost min distance considered
std::pair<int, double> RRTStarAR::FindClosestAndCost(const Node& from, const Node& to) {
    int closestIndex = -1;
    double minCost = std::numeric_limits<double>::max();

    for (int i = 0; i < nodes.size(); ++i) {
        if (&nodes[i] == &from) continue;

        double dist = std::sqrt(std::pow(to.x - nodes[i].x, 2) + std::pow(to.y - nodes[i].y, 2));
        double cost = nodes[i].cost + dist;

        if (cost < minCost && dist < MIN_NODE_DIST) {
            closestIndex = i;
            minCost = cost;
        }
    }

    return {closestIndex, minCost};
}

// Function to rewire the tree if a lower-cost path is found through the new node
bool RRTStarAR::Rewire(Node& newNode, int closestIndex) {
    double currentCost = newNode.cost;
    double minCost = currentCost;

    for (int i = 0; i < nodes.size(); ++i) {
        if (&nodes[i] == &newNode || &nodes[i] == &nodes[closestIndex]) continue;

        double dist = std::sqrt(std::pow(newNode.x - nodes[i].x, 2) + std::pow(newNode.y - nodes[i].y, 2));
        double cost = nodes[i].cost + dist;

        if (cost < minCost) {
            minCost = cost;
            newNode.parent = &nodes[i];
            currentCost = cost;
        }
    }

    return (currentCost < minCost);
}

bool RRTStarAR::IsGoalReached(const Node& node) const {
    // Check if the given node is close to any of the goal nodes
    double dist = std::sqrt(std::pow(node.x - goalX, 2) + std::pow(node.y - goalY, 2));
    if (dist < goalReachedRadius) {
        return true;
    }
    return false;
}

std::vector<Node> RRTStarAR::GetPath(){
    std::vector<Node> path;
    Node* node = &nodes.back();
    while (node != nullptr) {
        path.emplace_back(static_cast<int>(node->x), static_cast<int>(node->y), nullptr, 0.0);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Node> RRTStarAR::FindNeighboors(const Node& newNode){
    std::vector<Node> neighboors;
    for (const Node& node : nodes) {
        double dist = std::sqrt(std::pow(newNode.x - node.x, 2) + std::pow(newNode.y - node.y, 2));
        if (dist < MIN_NODE_DIST) {
            neighboors.push_back(node);
        }
    }
    return neighboors;
}

std::vector<std::pair<int, int>> RRTStarAR::BresenhamLine(int x1, int y1, int x2, int y2) {
    std::vector<std::pair<int, int>> line;

    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        line.emplace_back(x1, y1);

        if (x1 == x2 && y1 == y2) {
            break;
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }

    return line;
}
