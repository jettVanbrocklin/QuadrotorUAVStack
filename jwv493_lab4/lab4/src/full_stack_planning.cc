#include <cstdlib>
#include <vector>

#include <fstream>

#include "a_star2d.h"
#include "gnuplot-iostream.h"
#include "gui2d.h"
#include "occupancy_grid2d.h"
#include "path_info.h"
#include "polynomial_sampler.h"
#include "polynomial_solver.h"

using namespace game_engine;

void writeToCSVfile(std::string name, Eigen::MatrixXd matrix);

int main(int argc, char** argv) {
  if (argc != 6) {
    std::cerr << "Usage: ./full_stack_planning occupancy_grid_file row1 col1 "
                 "row2 col2"
              << std::endl;
    return EXIT_FAILURE;
  }

  // Parsing input
  const std::string occupancy_grid_file = argv[1];
  const std::shared_ptr<Node2D> start_ptr = std::make_shared<Node2D>(
      Eigen::Vector2i(std::stoi(argv[2]), std::stoi(argv[3])));
  const std::shared_ptr<Node2D> end_ptr = std::make_shared<Node2D>(
      Eigen::Vector2i(std::stoi(argv[4]), std::stoi(argv[5])));

  // Load an occupancy grid from a file
  OccupancyGrid2D occupancy_grid;
  occupancy_grid.LoadFromFile(occupancy_grid_file);

  // Transform an occupancy grid into a graph
  const Graph2D graph = occupancy_grid.AsGraph();

  /////////////////////////////////////////////////////////////////////////////
  // RUN A STAR
  // TODO: Run your A* implementation over the graph and nodes defined above.
  //       This section is intended to be more free-form. Using previous
  //       problems and examples, determine the correct commands to complete
  //       this problem. You may want to take advantage of some of the plotting
  //       and graphing utilities in previous problems to check your solution on
  //       the way.
  /////////////////////////////////////////////////////////////////////////////

  AStar2D a_star;
  PathInfo path_info = a_star.Run(graph, start_ptr, end_ptr);

  // Display the solution
  Gui2D gui;
  gui.LoadOccupancyGrid(&occupancy_grid);
  gui.LoadPath(path_info.path);
  gui.Display("AStar");

  // Print the solution
  path_info.details.Print();
  /////////////////////////////////////////////////////////////////////////////
  // RUN THE POLYNOMIAL PLANNER
  // TODO: Convert the A* solution to a problem the polynomial solver can
  //       solve. Solve the polynomial problem, sample the solution, figure out
  //       a way to export it to Matlab.
  /////////////////////////////////////////////////////////////////////////////

  std::vector<double> times = {};
  std::vector<p4::NodeEqualityBound> node_equality_bounds = {};

  for(int i = 0; i < path_info.details.path_length; i++){
    // add time for iteration
    times.push_back(i);

    // add the point to the node_equality_bounds
    int x = path_info.path[i]->Data().x();
    int y = path_info.path[i]->Data().y();
    node_equality_bounds.push_back(p4::NodeEqualityBound(0,i,0,x));
    node_equality_bounds.push_back(p4::NodeEqualityBound(1,i,0,y));
    
    // minimize acceleration (bound it to 0)
    // node_equality_bounds.push_back(p4::NodeEqualityBound(0,i,2,0));
    // node_equality_bounds.push_back(p4::NodeEqualityBound(1,i,2,0));
  }

  // Options to configure the polynomial solver with
  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 2;     // 2D
  solver_options.polynomial_order = 8;   // Fit an 8th-order polynomial
  solver_options.continuity_order = 4;   // Require continuity to the 4th order
  solver_options.derivative_order = 4;   // Minimize snap

  osqp_set_default_settings(&solver_options.osqp_settings);
  solver_options.osqp_settings.polish = true;       // Polish the solution, getting the best answer possible
  solver_options.osqp_settings.verbose = false;     // Suppress the printout

  // Use p4::PolynomialSolver object to solve for polynomial trajectories
  p4::PolynomialSolver solver(solver_options);
  const p4::PolynomialSolver::Solution path
    = solver.Run(
        times, 
        node_equality_bounds, 
        {}, 
        {});

  
  // Sampling for Matlab
  // Sample Position
  p4::PolynomialSampler::Options sampler_options_pos;
  sampler_options_pos.frequency = 200;             // Number of samples per second
  sampler_options_pos.derivative_order = 0;        // Derivative to sample (0 = pos)
  p4::PolynomialSampler sampler_pos(sampler_options_pos);
  Eigen::MatrixXd pos = sampler_pos.Run(times, path);
  writeToCSVfile("pos_samples", pos);

  // Sample Velocity
  p4::PolynomialSampler::Options sampler_options_vel;
  sampler_options_vel.frequency = 200;             // Number of samples per second
  sampler_options_vel.derivative_order = 1;        // Derivative to sample (0 = pos)
  p4::PolynomialSampler sampler_vel(sampler_options_vel);
  Eigen::MatrixXd vel = sampler_vel.Run(times, path);
  writeToCSVfile("vel_samples", vel);

  // Sample Acceleration
  p4::PolynomialSampler::Options sampler_options_acc;
  sampler_options_acc.frequency = 200;             // Number of samples per second
  sampler_options_acc.derivative_order = 2;        // Derivative to sample (0 = pos)
  p4::PolynomialSampler sampler_acc(sampler_options_acc);
  Eigen::MatrixXd acc = sampler_acc.Run(times, path);
  std::cout << "______________" << std::endl;
  std::cout << "Writing to acc_samples" << std::endl;
  writeToCSVfile("acc_samples", acc);
  std::cout << "Done Writing" << std::endl;
  std::cout << "______________" << std::endl;


  // // Sampling and Plotting
  { // Plot 2D position
    // Options to configure the polynomial sampler with
    p4::PolynomialSampler::Options sampler_options;
    sampler_options.frequency = 200;             // Number of samples per second
    sampler_options.derivative_order = 0;        // Derivative to sample (0 = pos)

    // Use this object to sample a trajectory
    p4::PolynomialSampler sampler(sampler_options);
    Eigen::MatrixXd samples = sampler.Run(times, path);

    // Plotting tool requires vectors
    std::vector<double> t_hist, x_hist, y_hist;
    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
      t_hist.push_back(samples(0,time_idx));
      x_hist.push_back(samples(1,time_idx));
      y_hist.push_back(samples(2,time_idx));
    }

    // gnu-iostream plotting library
    // Utilizes gnuplot commands with a nice stream interface
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Trajectory'" << std::endl;
      gp.send1d(boost::make_tuple(x_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'X'" << std::endl;
      gp << "set ylabel 'Y'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'X-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, x_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'X-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Y-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'Y-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
  }



  return EXIT_SUCCESS;
}

// Source - https://stackoverflow.com/q/18400596
// Posted by erogol, modified by community. See post 'Timeline' for change history
// Retrieved 2026-03-21, License - CC BY-SA 4.0

void writeToCSVfile(std::string name, Eigen::MatrixXd matrix)
{
  std::ofstream file(name.c_str());

  for(int  i = 0; i < matrix.rows(); i++){
      for(int j = 0; j < matrix.cols(); j++){
         std::string str = std::to_string(matrix(i,j));
         if(j+1 == matrix.cols()){
             file<<str;
         }else{
             file<<str<<' ';
         }
      }
      file<<'\n';
  }

  file.close();
}
