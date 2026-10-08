#include "a_star2d.h"

#include <queue>

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
  // Heuristic value from this node to end node
  double heuristic;

  // Equality operator
  bool operator==(const NodeWrapper& other) const {
    return *(this->node_ptr) == *(other.node_ptr);
  }
};

// Compares the values of two NodeWrapper pointers.  Necessary for the priority
// queue.
bool NodeWrapperPtrCompare(const std::shared_ptr<NodeWrapper>& lhs,
                           const std::shared_ptr<NodeWrapper>& rhs) {
  return lhs->cost + lhs->heuristic > rhs->cost + rhs->heuristic;
}

///////////////////////////////////////////////////////////////////
// EXAMPLE HEURISTIC FUNCTION
// YOU WILL NEED TO MODIFY THIS OR WRITE YOUR OWN FUNCTION
///////////////////////////////////////////////////////////////////
double manhatten_heuristic(const std::shared_ptr<Node2D>& current_ptr,
                 const std::shared_ptr<Node2D>& end_ptr) {
  double x_diff = std::abs(current_ptr->Data().x() - end_ptr->Data().x());
  double y_diff = std::abs(current_ptr->Data().y() - end_ptr->Data().y());
  return x_diff + y_diff;
}

double euclidean_heuristic(const std::shared_ptr<Node2D>& current_ptr,
                 const std::shared_ptr<Node2D>& end_ptr) {
  double x_diff = std::abs(current_ptr->Data().x() - end_ptr->Data().x());
  double y_diff = std::abs(current_ptr->Data().y() - end_ptr->Data().y());
  return std::sqrt(std::pow((x_diff),2) + std::pow((y_diff),2));
}

double underestimate_heuristic(const std::shared_ptr<Node2D>& current_ptr,
                 const std::shared_ptr<Node2D>& end_ptr) {
  return 0;
}

double overestimate_heuristic(const std::shared_ptr<Node2D>& current_ptr,
                 const std::shared_ptr<Node2D>& end_ptr) {
  double x_diff = std::abs(current_ptr->Data().x() - end_ptr->Data().x());
  double y_diff = std::abs(current_ptr->Data().y() - end_ptr->Data().y());
  return 2 * (x_diff + y_diff);
}

}  // namespace

int contains(std::vector<std::shared_ptr<NodeWrapper>> vec, std::shared_ptr<NodeWrapper> node_ptr);


PathInfo AStar2D::Run(const Graph2D& graph,
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
  std::priority_queue<
      NodeWrapperPtr, std::vector<NodeWrapperPtr>,
      std::function<bool(const NodeWrapperPtr&, const NodeWrapperPtr&)>>
      nodes_to_explore(NodeWrapperPtrCompare);

  std::vector<NodeWrapperPtr> explored_nodes;

  ///////////////////////////////////////////////////////////////////
  // YOUR WORK GOES BELOW
  // SOME EXAMPLE CODE PROVIDED
  ///////////////////////////////////////////////////////////////////

  std::unordered_map<Node2D*, double> best_cost;

  // Create a NodeWrapperPtr
  NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
  nw_ptr->parent = nullptr;
  nw_ptr->node_ptr = start_ptr;
  nw_ptr->cost = 0;
  nw_ptr->heuristic = euclidean_heuristic(start_ptr, end_ptr);
  nodes_to_explore.push(nw_ptr);

  NodeWrapperPtr goal_node;


  while(!nodes_to_explore.empty()){
    NodeWrapperPtr node_to_explore = nodes_to_explore.top();
    nodes_to_explore.pop();
    // int index_contained = contains(explored_nodes, node_to_explore);
    // if(index_contained != -1){
    //   if(node_to_explore->cost >= explored_nodes[index_contained]->cost){
    //     continue;
    //   }
    // }

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
      double new_cost = node_to_explore->cost + cost;
      // if there is a current value for this node in best cost or if a better cost was found
      if((best_cost.count(sink_ptr.get()) == 0) || new_cost < best_cost[sink_ptr.get()]){
        best_cost[sink_ptr.get()] = new_cost;
        NodeWrapperPtr neighbor_node = std::make_shared<NodeWrapper>();
        neighbor_node->parent = node_to_explore;
        neighbor_node->node_ptr = sink_ptr;
        neighbor_node->cost = new_cost;
        //Underestimation
        //neighbor_node->heuristic = underestimate_heuristic(sink_ptr, end_ptr);
        //Equal Estimation
        neighbor_node->heuristic = euclidean_heuristic(sink_ptr, end_ptr);
        //Equal Estimation
        //neighbor_node->heuristic = overestimate_heuristic(sink_ptr, end_ptr);

        nodes_to_explore.push(neighbor_node);
      }
      
    }
  }



  // Create a PathInfo
  PathInfo path_info;
  path_info.details.num_nodes_explored = explored_nodes.size();
  path_info.details.path_length = 0;
  path_info.details.path_cost = goal_node->cost;
  path_info.details.run_time = timer.Stop();
  path_info.path = {};

  // Push an example node to PathInfo.path.  Note that in your implementation,
  // path_info.path (which, as you can see in path_info.h, is just a vector of
  // pointers to Node2D objects), should contain the sequence of nodes
  // traversed from start_ptr to end_ptr.
  //path_info.path.push_back(nw_ptr->node_ptr);

  std::vector<std::shared_ptr<Node2D>> path_reversed = {};

  NodeWrapperPtr current_node = goal_node;
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
  // You must return a PathInfo
  return path_info;
}

int contains(std::vector<std::shared_ptr<NodeWrapper>> vec, std::shared_ptr<NodeWrapper> node_ptr){
  for (int i = 0; i < vec.size(); i++){
    if(*(vec[i]->node_ptr) == *(node_ptr->node_ptr)){ // operator== is assigned
      return i;
    }
  }
  return -1;
}

}  // namespace game_engine
