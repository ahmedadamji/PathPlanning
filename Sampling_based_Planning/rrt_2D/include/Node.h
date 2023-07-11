#ifndef NODE_H
#define NODE_H

class Node {
public:
    Node(std::pair<int, int> n) : x(n.first), y(n.second), parent(nullptr) {}

    int x, y;
    Node* parent;
};

#endif /* NODE_H */
