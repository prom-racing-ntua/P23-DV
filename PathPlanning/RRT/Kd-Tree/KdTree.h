#ifndef KdTree_H
#define KdTree_H

#include "../helper_funcs/helper.h"
#include "iostream"
#include "vector"

class KdTree {
    public:
    
    struct Node
    {
        Node *left, *right;
        Node *parent;
        double x;
        double y;
        double yaw;
        double cost;
        
        /**
        * @brief Default constructor of Node. 
        */
        Node();

        /**
        * @brief Constructor of Node. Initializes x,y,yaw.
        * 
        * @param coordX coordinate of Node.
        * @param coordY coordinate of Node.
        * @param Yaw of Node. 
        */
        Node ( double coordX, double coordY, double Yaw );
    };

    /**
     * @brief Default constructor of k-dimension tree.
    */
    KdTree ();

    /**
     * @brief Insert a new node to the tree.
     * 
     * @param node new node to insert to the tree. Of type struct Node.
     * 
     * @throws InvalidEntryError if `point` is not of length k.
     * @throws DuplicateEntryError if a point with the same value already exists inside the tree.
    */
    void insert ( Node *node );

    /**
     * @brief Nearest Neighbor Search.
     * 
     * @param query point to find the NN. Of type struct Point.
     * 
     * @return pointer to the node of the nearest neighbor to the point.
     * 
     * @throws TreeEmptyError when searching an empty tree.
     * @throws InvalidEntryError if `query` is not of length k.
    */
    Node *NNS ( Point *query );

    /**
     * @brief Euclidean distance.
     * 
     * @param a of type Point.
     * @param b of type Node.
     * 
     * @returns Euclidean distance between two points in space.
    */
    double distance ( Point *a, Node *b );

    // /**
    //  * @brief Choose parent for a certain node.
    //  * 
    //  * @param query point to choose it's parent node from the kdTree.
    //  * @param nn nearest neighbor of the query. Struct Node.
    //  * 
    //  * @returns a node.
    // */
    // Node *chooseParent ( Point *query, Node **nn );

    /**
     * @brief Get the leaf nodes of kd tree.
     * 
     * @param leaf_min_cost minimum cost of a node to be consider as a leaf.
     * 
     * @return a vector with all leaf nodes of type struct Node.
    */
    std::vector<Node*> findLeafNodes ( const double leaf_min_cost );

    private:

    const unsigned dimensions;
    Node *root;

    /**
     * @brief Create a new Node.
     * 
     * @param point point used to create the new node .Of type struct Point.
     * @param vec yaw and cost of new node. Of type struct Node. 
     * @param parent Parent of new node. Of type struct Node.  
     * 
     * @return pointer to the new Node.
    */
    Node *createNode ( Point *point, std::vector<double> vec, Node *parent );

    /**
     * @brief Insert a new node to the tree (recursive helper function).
     * 
     * @param root pointer to the root of kdTRee.
     * @param node new node to insert to the tree. Of type struct Node.
     * @param depth keep track of the depth during recursion. Initial call shouldn't provide a value.
    */
    void insert_help ( Node **root, Node *node, unsigned depth = 0 );

    /**
     * @brief Nearest Neighbor Search (recursive helper function).
     * 
     * @param query Point to find the NN of. Of type struct Point.
     * @param node of tree to search. Of type struct Node.
     * @param best_point reference to the currently closest point to the query.
     * @param best_dist reference to the currently closests distance to the query.
     * @param depth keep track of the depth during recursion. Initial call shouldn't provide a value.
    */
    void NNS_help ( Point *query, Node *node, Node **nearest_node, double &best_dist, unsigned depth = 0 );

    /**
     * @brief Helper function of finding all leaf nodes in the kd-Tree.
     * 
     * @param node to be checked if it is leaf or not.
     * @param leafNodes a vector with all leaf nodes.
     * @param leaf_min_cost minimum cost of a node to be consider as a leaf.
     * 
    */
    void findLeafNodes_help ( Node *node, std::vector<Node*>& leafNodes, const double leaf_min_cost );
};

#endif //KdTree_H