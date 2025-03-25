#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

mbot_lcm_msgs::path2D_t search_for_path(mbot_lcm_msgs::pose2D_t start,
                                             mbot_lcm_msgs::pose2D_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    bool found_path = false;
    
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    Node* startNode = new Node(startCell.x, startCell.y);
    Node* goalNode = new Node(goalCell.x, goalCell.y);
    
    startNode->g_cost = 0;
    startNode->h_cost = h_cost(startNode, goalNode, distances);

    PriorityQueue openList;
    openList.push(startNode);

    std::vector<Node*> closedList;

    while(!openList.empty()){
        Node* currentNode = openList.pop();
        closedList.push_back(currentNode);

        if(*currentNode == *goalNode){
            goalNode = currentNode;
            found_path = true;
            break;
        }

        auto children = expand_node(currentNode, distances, params);
        for(auto& child : children){
            if(is_in_list(child, closedList)){
                continue;
            }

            double tentative_g_cost = currentNode->g_cost + g_cost(child, currentNode, distances, params);

            if(openList.is_member(child)){
                Node* openNode = openList.get_member(child);
                if(tentative_g_cost < openNode->g_cost){
                    openNode->g_cost = tentative_g_cost;

                    openNode->h_cost = h_cost(child, goalNode, distances);;
                    openNode->parent = currentNode;
                }
            }
            else{
                child->g_cost = tentative_g_cost;
                child->h_cost = h_cost(child, goalNode, distances);
                child->parent = currentNode;
                openList.push(child);
            }
        }
    }

    mbot_lcm_msgs::path2D_t path;
    path.utime = start.utime;
    if (found_path)
    {
        auto nodePath = extract_node_path(goalNode, startNode);
        path.path = extract_pose_path(nodePath, distances);
        // Remove last pose, and add the goal pose
        path.path.pop_back();
        path.path.push_back(goal);
    }

    else printf("[A*] Didn't find a path\n");
    path.path_length = path.path.size();
    return path;
}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    double h_cost = 0.0;
    ////////////////// TODO: Implement your heuristic //////////////////////////
    double dx = abs(from->cell.x - goal->cell.x);
    double dy = abs(from->cell.y - goal->cell.y);
    h_cost = (dx + dy);

    return h_cost;
}

double g_cost(Node* child, Node* parent, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    double g_cost = 0.0;
    ////////////////// TODO: Implement your goal cost, use obstacle distances //////////////////////////
    double cellDistance = distances(child->cell.x, child->cell.y);
    double diff = abs(distances(child->cell.x, child->cell.y) - distances(parent->cell.x, parent->cell.y));
    // double diff = 1.0;
    if(cellDistance > params.minDistanceToObstacle && cellDistance < params.maxDistanceWithCost)
    {
        g_cost = diff + pow(params.maxDistanceWithCost - cellDistance, params.distanceCostExponent);
    }
    else if(cellDistance < params.minDistanceToObstacle)
    {
        g_cost = std::numeric_limits<double>::infinity();
    }
    else
    {
        g_cost = diff;
    }

    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    std::vector<Node*> children;
    ////////////////// TODO: Implement your expand node algorithm //////////////////////////
    const int xDeltas[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int yDeltas[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    for(int n = 0; n < 8; ++n){
        int x = node->cell.x + xDeltas[n];
        int y = node->cell.y + yDeltas[n];
        if(distances.isCellInGrid(x, y) && distances(x, y) > params.minDistanceToObstacle){
            Node* child = new Node(x, y);
            children.push_back(child);
        }
    }

    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    std::vector<Node*> path;
    ////////////////// TODO: Implement your extract node function //////////////////////////
    // Traverse nodes and add parent nodes to the vector
    Node* node = goal_node;
    while(node != NULL){
        path.push_back(node);
        if (*node == *start_node) break;
        node = node->parent;
    }
    
    // Reverse path
    std::reverse(path.begin(), path.end());
    return path;
}

// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose2D_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    std::vector<mbot_lcm_msgs::pose2D_t> path;
    ////////////////// TODO: Implement your extract_pose_path function //////////////////////////
    // This should turn the node path into a vector of poses (with heading) in the global frame
    // You should prune the path to get a waypoint path suitable for sending to motion controller
    std::vector<Node*> pruned_path = prune_node_path(nodes);
    for (auto &&node : pruned_path)
    {
        mbot_lcm_msgs::pose2D_t pose;
        pose.x = distances.originInGlobalFrame().x + node->cell.x * distances.metersPerCell();
        pose.y = distances.originInGlobalFrame().y + node->cell.y * distances.metersPerCell();
        pose.theta = 0.0;
        path.push_back(pose);
    }
    
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;

}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath)
{
    std::vector<Node*> new_node_path;
    ////////////////// TODO: Optionally implement a prune_node_path function //////////////////////////
    // This should remove points in the path along the same line
    for(auto& node : nodePath){
        if(new_node_path.size() < 2){
            new_node_path.push_back(node);
        }
        else{
            Node* lastNode = new_node_path.back();
            Node* secondLastNode = new_node_path[new_node_path.size()-2];
            if(((node->cell.y - lastNode->cell.y) * (lastNode->cell.x - secondLastNode->cell.x) - (node->cell.x - lastNode->cell.x) * (lastNode->cell.y - secondLastNode->cell.y)) == 0){
                new_node_path.pop_back();
            }
            new_node_path.push_back(node);
        }
    }

    int path_size = new_node_path.size();
    for(int i = 1; i < path_size - 1; i++)
    {
        Node* node = new_node_path[i];
        Node* lastNode = new_node_path[i-1];
        Node* nextNode = new_node_path[i+1];
        float dist_nodes = sqrt(pow(node->cell.x - lastNode->cell.x, 2) + pow(node->cell.y - lastNode->cell.y, 2)) 
                            + sqrt(pow(node->cell.x - nextNode->cell.x, 2) + pow(node->cell.y - nextNode->cell.y, 2));
        float dist_last_next = sqrt(pow(lastNode->cell.x - nextNode->cell.x, 2) + pow(lastNode->cell.y - nextNode->cell.y, 2));
        if(dist_nodes >= dist_last_next && dist_last_next < 6.0){
            new_node_path.erase(new_node_path.begin() + i);
            path_size--;
            i--;
        }
    }

    return new_node_path;

}
