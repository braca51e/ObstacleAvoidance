//
// Created by Luis Bracamontes on 10/25/23.
//

#include "RrtStar.h"

void RRTStarAR::GenerateRRTStarAR() {

    // Initialize the tree with the starting node
    nodes.push_back({startX, startY, nullptr, 0.0});

    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        Node randomNode = GenerateRandomNode();
        Node nearestNode = FindNearestNode(randomNode);
        // Attempt to steer from the nearest node towards the random node
        Node newNode = Steer(nearestNode, randomNode);
        // Check if the new node is valid (not in collision) @TODO: Check collision with a line instead of a point?
        if (IsValidNode(newNode)) {
            std::pair<int, double> nearest = FindClosestAndCost(nearestNode, newNode);
            int closestIndex = nearest.first;
            double minCost = nearest.second;
            if (closestIndex < 0) {
                continue;
            }
            // Assign the parent for newNode here, after validation
            newNode.parent = &nodes[closestIndex];

            if (minCost > 0) {
                newNode.cost = newNode.parent->cost + minCost;
                // Apply the closeNodePenalty if newNode and its parent share the same parent node
                if (newNode.parent == nodes[closestIndex].parent) {
                    newNode.cost += closeNodePenalty;
                }
                if (Rewire(newNode, closestIndex)) {
                    nodes[closestIndex].parent = &nodes.back();
                }
            }

            nodes.push_back(newNode);

            if (IsGoalReached(newNode)) {
                // You have reached the goal, you can exit or further process the path
                break;
            }
        }
    }
}

std::vector<std::vector<Node>> RRTStarAR::GenerateNPathsRRTStarAR(int N) {
    // Create a single goal node
    Node goalNode = {goalX, goalY, nullptr, 0.0};
    std::vector<std::vector<Node>> paths;
    for (int n = 0; n < N; ++n) {
        // Reset the tree for each path
        nodes.clear();
        nodes.push_back({startX, startY, nullptr, 0.0});

        for (int iteration = 0; iteration < maxIterations; ++iteration) {
            Node randomNode = GenerateRandomNode();
            Node nearestNode = FindNearestNode(randomNode);

            Node newNode = Steer(nearestNode, randomNode);

            if (IsValidNode(newNode)) {
                std::pair<int, double> nearest = FindClosestAndCost(nearestNode, newNode);
                int closestIndex = nearest.first;
                double minCost = nearest.second;

                if (minCost > 0) {
                    newNode.cost = newNode.parent->cost + minCost;
                    if (Rewire(newNode, closestIndex)) {
                        nodes[closestIndex].parent = &nodes.back();
                    }
                }
                nodes.push_back(newNode);
            }
        }
        paths.emplace_back(nodes);
    }

    return paths;
}

// Function to generate a random node within the map bounds
Node RRTStarAR::GenerateRandomNode() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> xDist(0, map.rows - 1);
    std::uniform_real_distribution<double> yDist(0, map.cols - 1);

    double x = xDist(gen);
    double y = yDist(gen);

    return {x, y, nullptr, 0.0};
}

// Function to find the nearest node in the tree to the given random node
Node RRTStarAR::FindNearestNode(const Node& randomNode) {
    Node nearestNode = nodes[0];
    double minDist = std::sqrt(std::pow(randomNode.x - nearestNode.x, 2) + std::pow(randomNode.y - nearestNode.y, 2));

    for (const Node& node : nodes) {
        double dist = std::sqrt(std::pow(randomNode.x - node.x, 2) + std::pow(randomNode.y - node.y, 2));
        if (dist < minDist) {
            nearestNode = node;
            minDist = dist;
        }
    }

    return nearestNode;
}

// Function to steer from the 'from' node towards the 'to' node with a specified step size
Node RRTStarAR::Steer(const Node& from, const Node& to) {
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist <= stepSize) {
        return to;
    }

    double ratio = stepSize / dist;
    double newX = from.x + dx * ratio;
    double newY = from.y + dy * ratio;

    return {newX, newY, nullptr, 0.0};
}

// Function to check if a node is valid (not in collision with the map)
bool RRTStarAR::IsValidNode(const Node& node) {
    // Implement collision checking using your cv::Mat map
    // You will need to adapt this based on your specific map representation
    int x = static_cast<int>(node.x);
    int y = static_cast<int>(node.y);
    double mean_pix_value = (map.at<cv::Vec3b>(x, y)[0] + map.at<cv::Vec3b>(x, y)[1] +
            map.at<cv::Vec3b>(x, y)[2])/3.0;
    mean_pix_value /= 255.0;
    return (x >= 0 && x < map.cols && y >= 0 && y < map.rows && mean_pix_value > tau_obs_);
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

bool RRTStarAR::IsGoalReached(const Node& node) {
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
        node->x = static_cast<int>(node->x);
        node->y = static_cast<int>(node->y);
        path.push_back(*node);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}