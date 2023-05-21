#include "KdTree.h"
#include "exceptions.cpp"
#include <cstdio>
#include <cmath>
#include <iostream>

KdTree::Node::Node () : x(), y(), yaw(), cost(), left(), right(), parent() {}

KdTree::Node::Node ( double coordX, double coordY, double Yaw ) : x(coordX), y(coordY), yaw(Yaw), cost(0), left(nullptr), right(nullptr), parent(nullptr) {}

KdTree::KdTree () : dimensions(2)
{
    this->root = nullptr;
}

KdTree::Node *KdTree::NNS ( Point *query ) 
{
    if (this->root == nullptr) {
        throw TreeEmptyError();
    }
    // initialize an "empty" Node
    Point zeros(0, 0);
    std::vector<double> vec{0, 0};  //yaw=0, cost=0
    KdTree::Node *nearest_node = this->createNode(&zeros, vec, nullptr);

    double best_dist = __INT_MAX__;

    this->NNS_help(query, this->root, &nearest_node, best_dist=__INT_MAX__);

    return nearest_node;
}

void KdTree::NNS_help ( Point *query, KdTree::Node *node, KdTree::Node **nearest_node, double &best_dist, unsigned depth )
{
    double dist = this->distance(query, node);
    if (dist < best_dist) {
        best_dist = dist;
        *nearest_node = node;
    }

    unsigned current_depth = depth % dimensions;
    double query_depth_coord;
    double parent_depth_coord;
    
    if (current_depth == 0) {
        query_depth_coord = query->x;
        parent_depth_coord = node->x;
    } else if (current_depth == 1) {
        query_depth_coord = query->y;
        parent_depth_coord = node->y;
    }

    if (node->left == nullptr && node->right == nullptr) return;  // leaf case
    if (query_depth_coord <= parent_depth_coord) {
        if (node->left != nullptr) {
            this->NNS_help(query, node->left, nearest_node, best_dist, depth + 1);
        }
        if ((query_depth_coord + best_dist > parent_depth_coord) && (node->right != nullptr)) {
            this->NNS_help(query, node->right, nearest_node, best_dist, depth + 1);
        }
    }   
    else {
        if (node->right != nullptr) {
            this->NNS_help(query, node->right, nearest_node, best_dist, depth + 1);
        }
        if (query_depth_coord - best_dist <= parent_depth_coord && node->left != nullptr) {
            this->NNS_help(query, node->left, nearest_node, best_dist, depth + 1);
        }
    }
}

KdTree::Node *KdTree::createNode ( Point *point, std::vector<double> vec, Node *parent ) 
{
    Node *node = new Node();
    node->left = node->right = nullptr;
    node->parent = parent;
    node->x = point->x;
    node->y = point->y;
    node->yaw = vec[0];
    node->cost = vec[1];

    return node;
}

double KdTree::distance ( Point *a, Node *b ) 
{
    double sum = pow(a->x - b->x, 2) + pow(a->y - b->y, 2);

    return sqrt(sum);
}

void KdTree::insert ( KdTree::Node *node )
{
    this->insert_help(&this->root, node);
}

void KdTree::insert_help ( KdTree::Node **root, KdTree::Node *node, unsigned depth )
{
    if (*root == nullptr) {
        *root = node;
        return;
    }

    unsigned current_depth = depth % dimensions;
    double child_depth_coord;
    double parent_depth_coord;

    if (current_depth == 0) {
        child_depth_coord = node->x;
        parent_depth_coord = (*root)->x;
    } else if (current_depth == 1) {
        child_depth_coord = node->y;
        parent_depth_coord = (*root)->y;
    }
    
    if (child_depth_coord < parent_depth_coord) {
        this->insert_help(&(*root)->left, node, depth + 1);
    } else {
        this->insert_help(&(*root)->right, node, depth + 1);
    }

    // if (child_depth_coord < parent_depth_coord) {
    //     if ((*root)->left == nullptr) {
    //         std::cout << "yes" << std::endl;
    //         this->insert_help(&(*root)->left, node, depth + 1);
    //     } else if ((*root)->right == nullptr) {
    //         std::cout << "no" << std::endl;
    //         double dif = parent_depth_coord - child_depth_coord;
    //         if (current_depth == 0) {
    //             node->x = node->x + dif + 0.1;
    //         } else {
    //             node->y = node->y + dif + 0.1;
    //         }
    //         this->insert_help(&(*root)->right, node, depth + 1);
    //     }
    // } else {
    //     if ((*root)->right == nullptr) {
    //         std::cout << "yes" << std::endl;
    //         this->insert_help(&(*root)->right, node, depth + 1);
    //     } else if ((*root)->left == nullptr) {
    //         std::cout << "no" << std::endl;
    //         double dif = child_depth_coord - parent_depth_coord;
    //         if (current_depth == 0) {
    //             node->x = node->x - dif - 0.1;
    //         } else {
    //             node->y = node->y - dif - 0.1;
    //         }
    //         this->insert_help(&(*root)->left, node, depth + 1);
    //     }
    // }
}

std::vector<KdTree::Node*> KdTree::findLeafNodes ( const double leaf_min_cost )
{
    std::vector<KdTree::Node*> leafNodes;

    this->findLeafNodes_help(this->root, leafNodes, leaf_min_cost);

    return leafNodes;
}

void KdTree::findLeafNodes_help ( KdTree::Node *node, std::vector<KdTree::Node*>& leafNodes, const double leaf_min_cost ) 
{
    if (node == nullptr) return;
    // std::cout << node->left << "," << node->right << std::endl;

    if (node->left == nullptr && node->right == nullptr) {    
        // if (node->cost >= leaf_min_cost) {
        //     leafNodes.push_back(node);
        //     return;
        // }
        leafNodes.push_back(node);
        return;
    }

    this->findLeafNodes_help(node->left, leafNodes, leaf_min_cost);
    this->findLeafNodes_help(node->right, leafNodes, leaf_min_cost);
}