//
// Created by Luis Bracamontes on 10/25/23.
//

#include "RrtStar.h"

void RRTStarAR::GenerateRRTStarAR() {

    // Initialize the tree with the starting node
    nodes.clear();
    nodes.push_back({startX, startY, -1, 0.0});

    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        Node randomNode = GenerateRandomNode();
        int nearestNode = FindNearestNode(randomNode);
        // Attempt to steer from the nearest node towards the random node
        Node newNode = Steer(nearestNode, randomNode);
        // Check if the new node is valid (not in collision)
        if (CollisionFree(nearestNode, newNode)) {
            std::vector<Node> neighbors = FindNeighbors(newNode);
            std::pair<int, double> pair_search = FindClosestAndCost(neighbors, newNode, nearestNode);
            newNode.parent = pair_search.first;
            newNode.cost = pair_search.second;
            // Rewire the tree with the new node
            Rewire(neighbors, newNode, pair_search.first);
        }
        nodes.push_back(newNode);
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

    return {x, y, -1, 0.0};
}

// Function to find the nearest node in the tree to the given random node
int RRTStarAR::FindNearestNode(const Node& randomNode) {
    // Assign max value
    double minDist = std::numeric_limits<double>::max();
    int nearestNode = 0;

    for (int i = 0; i < nodes.size(); ++i) {
        double dist = std::sqrt(std::pow(randomNode.x - nodes[i].x, 2) + std::pow(randomNode.y - nodes[i].y, 2));
        if (dist < minDist) {
            nearestNode = i;
            minDist = dist;
        }
    }
    return nearestNode;
}

// Function to steer from the 'from' node towards the 'to' node with a specified step size
Node RRTStarAR::Steer(const int& from, const Node& to) const {
    double dx = to.x - nodes[from].x;
    double dy = to.y - nodes[from].y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist <= stepSize) {
        return to;
    }

    double ratio = stepSize / dist;
    double newX = nodes[from].x + dx * ratio;
    double newY = nodes[from].y + dy * ratio;

    return {newX, newY, -1, 0.0};
}

// Function to check if a node is valid (not in collision with the map)
//bool RRTStarAR::IsValidNode(const Node& node) {
bool RRTStarAR::CollisionFree(const int& nearestNode, const Node& newNode) {
    int x1 = static_cast<int>(nodes[nearestNode].x);
    int y1 = static_cast<int>(nodes[nearestNode].y);
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
std::pair<int, double> RRTStarAR::FindClosestAndCost(const std::vector<Node>& neighbors,  const Node& newNode, int nearest) {
    double minCost = nodes[nearest].cost + std::sqrt(std::pow(nodes[nearest].x - newNode.x, 2) + std::pow(nodes[nearest].y - newNode.y, 2));

    for (int i = 0; i < neighbors.size(); ++i) {
        if (CollisionFree(i, newNode) &&
            (neighbors[i].cost + std::sqrt(std::pow(nodes[i].x - newNode.x, 2) + std::pow(nodes[i].y - newNode.y, 2))) <
            minCost) {
            nearest = i;
            minCost = neighbors[i].cost +
                      std::sqrt(std::pow(nodes[i].x - newNode.x, 2) + std::pow(nodes[i].y - newNode.y, 2));
        }
    }
    return {nearest, minCost};
}

// Function to rewire the tree if a lower-cost path is found through the new node
void RRTStarAR::Rewire(const std::vector<Node>& neighbors, Node& newNode, int closestIndex) {
    double currentCost = newNode.cost;

    for (int i = 0; i < neighbors.size(); ++i) {
        if (CollisionFree(i, newNode) &&
            (newNode.cost + std::sqrt(std::pow(nodes[i].x - newNode.x, 2) + std::pow(nodes[i].y - newNode.y, 2))) <
            nodes[i].cost) {
            // @TODO I'm assigning an index that does not exist yet as new node has not been added to the tree
            nodes[i].parent = nodes.size();
            nodes[i].cost = newNode.cost + std::sqrt(std::pow(nodes[i].x - newNode.x, 2) + std::pow(nodes[i].y - newNode.y, 2));
        }
    }
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
    while (node->parent >= 0) {
        path.emplace_back(static_cast<int>(node->x), static_cast<int>(node->y), -1, 0.0);
        node = &nodes[node->parent];
    }
    // Add last point
    path.emplace_back(static_cast<int>(node->x), static_cast<int>(node->y), -1, 0.0);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Node> RRTStarAR::FindNeighbors(const Node& newNode){
    std::vector<Node> neighbors;
    for (const Node& node : nodes) {
        double dist = std::sqrt(std::pow(newNode.x - node.x, 2) + std::pow(newNode.y - node.y, 2));
        if (dist < MIN_NODE_DIST) {
            neighbors.push_back(node);
        }
    }
    return neighbors;
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
