#ifndef NODE_H
#define NODE_H

#include <memory>

class Node {
public:
    Node() : x(0), y(0), parent(nullptr) {}  // Default constructor
    Node(std::pair<double, double> n) : x(n.first), y(n.second), parent(nullptr) {}

    double x, y; // Node coordinates.
    std::shared_ptr<Node> parent; // Parent node.
};

#endif /* NODE_H */
