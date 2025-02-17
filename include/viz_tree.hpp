#pragma once

#include <iostream>
#include <vector>
#include <string>

// Define the structure for a general tree node

namespace tree_viz{
    struct Node {
        int data;
        std::vector<Node*> children;  // General tree: each node can have any number of children

        Node(int value) : data(value) {}
    };
}

void generateDot(std::ofstream &dotFile, tree_viz::Node* root) {
    if (!root) return;
    for (tree_viz::Node* child : root->children) {
        dotFile << "    " << root->data << " -> " << child->data << ";" << std::endl;
        generateDot(dotFile, child);
    }
}
