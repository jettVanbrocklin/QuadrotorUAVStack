#include "depth_first_search2d.h"

#include <stack>
#include <iostream>



namespace game_engine {

// Anonymous namespace. Put any file-local functions and variables within this
// scope
namespace {
// The NodeWrapper object can be used to form a linked list representing a path.
struct NodeWrapper {
  // Pointer to Node2D object
  std::shared_ptr<Node2D> node_ptr;
  // Cost to reach the node pointed to by node_ptr
  double cost;
  // Parent NodeWrapper object
  std::shared_ptr<struct NodeWrapper> parent;

  // Equality operator function
  bool operator==(const NodeWrapper& other) const {
    return *(this->node_ptr) == *(other.node_ptr);
  }
};
}  // namespace

bool contains(std::vector<std::shared_ptr<NodeWrapper>> vec, std::shared_ptr<NodeWrapper> node_ptr);

PathInfo DepthFirstSearch2D::Run(const Graph2D& graph,
                                 const std::shared_ptr<Node2D> start_ptr,
                                 const std::shared_ptr<Node2D> end_ptr) {
  using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;

  ///////////////////////////////////////////////////////////////////
  // SETUP
  // DO NOT MODIFY THIS
  ///////////////////////////////////////////////////////////////////
  Timer timer;
  timer.Start();

  // Use these data structures
  std::stack<NodeWrapperPtr> nodes_to_explore;
  std::vector<NodeWrapperPtr> explored_nodes;

  ///////////////////////////////////////////////////////////////////
  // YOUR WORK GOES BELOW
  // SOME EXAMPLE CODE PROVIDED
  ///////////////////////////////////////////////////////////////////

  //////////////////////////
  // Create a NodeWrapperPtr
  NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
  nw_ptr->parent = nullptr;
  nw_ptr->node_ptr = start_ptr;
  nw_ptr->cost = 0;
  nodes_to_explore.push(nw_ptr);
  //////////////////////////

  NodeWrapperPtr goal_node;

  while(!nodes_to_explore.empty()){
    NodeWrapperPtr node_to_explore = nodes_to_explore.top();
    // std::cout << "____________________________" << std::endl;
    // std::cout << "Current_Node: " << node_to_explore->node_ptr->Data() << std::endl;

    nodes_to_explore.pop();
    if(contains(explored_nodes, node_to_explore)){
      // std::cout<<"CONTAINS"<<std::endl;
      continue;
    }
    if(*(node_to_explore->node_ptr) == *end_ptr){
      goal_node = node_to_explore;
      break;
    }
    explored_nodes.push_back(node_to_explore);

    const std::vector<DirectedEdge2D> edges = graph.Edges(node_to_explore->node_ptr);
    // Iterate through the list of edges
    for(const auto edge : edges) {
      const auto source_ptr = edge.Source();
      const auto sink_ptr = edge.Sink();
      const double cost = edge.Cost();

      NodeWrapperPtr neighbor_node = std::make_shared<NodeWrapper>();
      neighbor_node->parent = node_to_explore;
      neighbor_node->node_ptr = sink_ptr;
      neighbor_node->cost = node_to_explore->cost + cost; // Not sure where cost comes from?
      nodes_to_explore.push(neighbor_node);
    }
  }

  //////////////////////////
  // Create a PathInfo
  PathInfo path_info;
  path_info.details.num_nodes_explored = explored_nodes.size();
  path_info.details.path_length = 0;
  path_info.details.path_cost = goal_node->cost;
  path_info.details.run_time = timer.Stop();
  path_info.path = {};
  //////////////////////////

  // reverse linked list to generate path
  // The last node_wrapper in explored_nodes should be neighbors with the end node
  
  std::vector<std::shared_ptr<Node2D>> path_reversed = {end_ptr};

  NodeWrapperPtr current_node = explored_nodes.back();
  path_info.details.path_length++;
  while(!(current_node == nw_ptr)){
    path_info.details.path_length++;
    path_reversed.push_back(current_node->node_ptr);
    current_node = current_node->parent;
  }
  path_info.details.path_length++;
  path_reversed.push_back(current_node->node_ptr);

  // Reverse found path
  int path_length = path_info.details.path_length;

  for(int i = 0; i < path_length; i++){
    path_info.path.push_back(path_reversed[path_length - 1 - i]); //reverses array
  }


  //////////////////////////  
  // Push an example node to PathInfo.path.  Note that in your implementation,
  // path_info.path (which, as you can see in path_info.h, is just a vector of
  // pointers to Node2D objects), should contain the sequence of nodes
  // traversed from start_ptr to end_ptr.
  // path_info.path.push_back(nw_ptr->node_ptr); // example
  //////////////////////////

  // You must return a PathInfo
  return path_info;
}

bool contains(std::vector<std::shared_ptr<NodeWrapper>> vec, std::shared_ptr<NodeWrapper> node_ptr){
  for (int i = 0; i < vec.size(); i++){
    if(*(vec[i]->node_ptr) == *(node_ptr->node_ptr)){ // operator== is assigned
      return true;
    }
  }
  return false;
}

}  // namespace game_engine
