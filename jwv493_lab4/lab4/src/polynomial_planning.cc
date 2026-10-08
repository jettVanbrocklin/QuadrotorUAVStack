#include <cstdlib>
#include <vector>

#include <cmath>

#include "polynomial_solver.h"
#include "polynomial_sampler.h"
#include "gnuplot-iostream.h"

///////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPES
// DO NOT MODIFY
///////////////////////////////////////////////////////////////////
void Example();
void DerivativeExperiments();
void ArrivalTimeExperiments();
void NumWaypointExperiments();

///////////////////////////////////////////////////////////////////
// MAIN FUNCTION
// TODO: UNCOMMENT THE FUNCTIONS YOU WANT TO RUN
///////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {
  // Example();
  // DerivativeExperiments();
  // ArrivalTimeExperiments();
  NumWaypointExperiments();

  return EXIT_SUCCESS;
}

// Example function that demonstrates how to use the polynomial solver. This
// example creates waypoints in a triangle: (0,0) -- (1,0) -- (1,1) -- (0,0)
void Example() {
  // Time in seconds
  const std::vector<double> times = {0,1,2,3};

  // The parameter order for p4::NodeEqualityBound is:
  // (dimension_index, node_idx, derivative_idx, value)
  const std::vector<p4::NodeEqualityBound> node_equality_bounds = {
    // The first node must constrain position, velocity, and acceleration
    p4::NodeEqualityBound(0,0,0,0),
    p4::NodeEqualityBound(1,0,0,0),
    p4::NodeEqualityBound(0,0,1,0),
    p4::NodeEqualityBound(1,0,1,0),
    p4::NodeEqualityBound(0,0,2,0),
    p4::NodeEqualityBound(1,0,2,0),

    // The second node constrains position
    p4::NodeEqualityBound(0,1,0,1),
    p4::NodeEqualityBound(1,1,0,0),

    // The third node constrains position
    p4::NodeEqualityBound(0,2,0,1),
    p4::NodeEqualityBound(1,2,0,1),

    // The fourth node constrains position
    p4::NodeEqualityBound(0,3,0,0),
    p4::NodeEqualityBound(1,3,0,0),
  };

  // Options to configure the polynomial solver with
  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 2;     // 2D
  solver_options.polynomial_order = 8;   // Fit an 8th-order polynomial
  solver_options.continuity_order = 4;   // Require continuity to the 4th order
  solver_options.derivative_order = 2;   // Minimize the 2nd order (acceleration)

  osqp_set_default_settings(&solver_options.osqp_settings);
  solver_options.osqp_settings.polish = true;       // Polish the solution, getting the best answer possible
  solver_options.osqp_settings.verbose = true;     // Suppress the printout

  // Use p4::PolynomialSolver object to solve for polynomial trajectories
  p4::PolynomialSolver solver(solver_options);
  const p4::PolynomialSolver::Solution path
    = solver.Run(
        times, 
        node_equality_bounds, 
        {}, 
        {});

  // Sampling and Plotting
  { // Plot 2D position
    // Options to configure the polynomial sampler with
    p4::PolynomialSampler::Options sampler_options;
    sampler_options.frequency = 100;             // Number of samples per second
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
}

void DerivativeExperiments() {

  // Time in seconds
  // TODO: SET THE TIMES FOR THE WAYPOINTS
  const std::vector<double> times = {0, 1, 2, 3, 4};

  // The parameter order for p4::NodeEqualityBound is:
  // (dimension_index, node_idx, derivative_idx, value)
  const std::vector<p4::NodeEqualityBound> node_equality_bounds = {
    ///////////////////////////////////////////////////////////////////
    // TODO: CREATE A SQUARE TRAJECTORY
    ///////////////////////////////////////////////////////////////////

    // first node: pos, vel
    p4::NodeEqualityBound(0,0,0,0),
    p4::NodeEqualityBound(1,0,0,0),

    // second node: pos, vel
    p4::NodeEqualityBound(0,1,0,1),
    p4::NodeEqualityBound(1,1,0,0),
    // third node: pos, vel
    p4::NodeEqualityBound(0,2,0,1),
    p4::NodeEqualityBound(1,2,0,1),
    // fourth node: pos, vel
    p4::NodeEqualityBound(0,3,0,0),
    p4::NodeEqualityBound(1,3,0,1),

    //back to start
    p4::NodeEqualityBound(0,4,0,0),
    p4::NodeEqualityBound(1,4,0,0),

  };

  // Options to configure the polynomial solver with
  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 2;     // 2D
  solver_options.polynomial_order = 8;   // Fit an 8th-order polynomial
  solver_options.continuity_order = 4;   // Require continuity to the 4th order
  solver_options.derivative_order = 1;   // TODO: VARY THE DERIVATIVE ORDER

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

  // Sampling and Plotting
  { // Plot 2D position
    // Options to configure the polynomial sampler with
    p4::PolynomialSampler::Options sampler_options;
    sampler_options.frequency = 100;             // Number of samples per second
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
}

void ArrivalTimeExperiments() {
  // Time in seconds
  // TODO: SET THE TIMES FOR THE WAYPOINTS
  //const std::vector<double> times = {0,1,2,3,4}; // Reasonable
  const std::vector<double> times = {0,0.05,0.1,0.15,0.2}; // Unreasonable
  //const std::vector<double> times = {0,3.3,6.6,9.9,13.2};; // Optimized


  // The parameter order for p4::NodeEqualityBound is:
  // (dimension_index, node_idx, derivative_idx, value)
  const std::vector<p4::NodeEqualityBound> node_equality_bounds = {
    ///////////////////////////////////////////////////////////////////
    // TODO: CREATE A SQUARE TRAJECTORY
    ///////////////////////////////////////////////////////////////////

    p4::NodeEqualityBound(0,0,0,0),
    p4::NodeEqualityBound(1,0,0,0),
    p4::NodeEqualityBound(0,0,1,0),
    p4::NodeEqualityBound(1,0,1,0),
    p4::NodeEqualityBound(0,0,2,0),
    p4::NodeEqualityBound(1,0,2,0),
    // second node:
    p4::NodeEqualityBound(0,1,0,1),
    p4::NodeEqualityBound(1,1,0,0),
    // third node:
    p4::NodeEqualityBound(0,2,0,1),
    p4::NodeEqualityBound(1,2,0,1),
    // fourth node:
    p4::NodeEqualityBound(0,3,0,0),
    p4::NodeEqualityBound(1,3,0,1),
    //back to start
    p4::NodeEqualityBound(0,4,0,0),
    p4::NodeEqualityBound(1,4,0,0),
    p4::NodeEqualityBound(0,4,1,0),
    p4::NodeEqualityBound(1,4,1,0),
    p4::NodeEqualityBound(0,4,2,0),
    p4::NodeEqualityBound(1,4,2,0),
  };

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

  // Sampling and Plotting
  { // Plot 2D position
    // Options to configure the polynomial sampler with
    p4::PolynomialSampler::Options sampler_options;
    sampler_options.frequency = 100;             // Number of samples per second
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
  { // Plot velocity
  p4::PolynomialSampler::Options sampler_options;
  sampler_options.frequency = 100;
  sampler_options.derivative_order = 1; // velocity

  p4::PolynomialSampler sampler(sampler_options);
  Eigen::MatrixXd samples = sampler.Run(times, path);

  std::vector<double> t_hist, vx_hist, vy_hist, v_mag;

  for(size_t i = 0; i < samples.cols(); ++i) {
    double t = samples(0,i);
    double vx = samples(1,i);
    double vy = samples(2,i);

    t_hist.push_back(t);
    vx_hist.push_back(vx);
    vy_hist.push_back(vy);
    v_mag.push_back(std::sqrt(vx*vx + vy*vy)); // speed
  }

  Gnuplot gp;
  gp << "plot '-' using 1:2 with lines title 'Speed |v|'" << std::endl;
  gp.send1d(boost::make_tuple(t_hist, v_mag));
  gp << "set grid" << std::endl;
  gp << "set xlabel 'Time (s)'" << std::endl;
  gp << "set ylabel 'Speed (m/s)'" << std::endl;
  gp << "replot" << std::endl;
  }
  { // Plot acceleration
  p4::PolynomialSampler::Options sampler_options;
  sampler_options.frequency = 100;
  sampler_options.derivative_order = 2; // acceleration

  p4::PolynomialSampler sampler(sampler_options);
  Eigen::MatrixXd samples = sampler.Run(times, path);

  std::vector<double> t_hist, ax_hist, ay_hist, a_mag;

  for(size_t i = 0; i < samples.cols(); ++i) {
    double t = samples(0,i);
    double ax = samples(1,i);
    double ay = samples(2,i);

    t_hist.push_back(t);
    ax_hist.push_back(ax);
    ay_hist.push_back(ay);
    a_mag.push_back(std::sqrt(ax*ax + ay*ay)); // magnitude
  }

  Gnuplot gp;
  gp << "plot '-' using 1:2 with lines title 'Acceleration |a|'" << std::endl;
  gp.send1d(boost::make_tuple(t_hist, a_mag));
  gp << "set grid" << std::endl;
  gp << "set xlabel 'Time (s)'" << std::endl;
  gp << "set ylabel 'Acceleration (m/s^2)'" << std::endl;
  gp << "replot" << std::endl;
}
}

void NumWaypointExperiments() {
  // Time in seconds
  // TODO: SET THE TIMES FOR THE WAYPOINTS

  std::vector<double> times = {};

  // The parameter order for p4::NodeEqualityBound is:
  // (dimension_index, node_idx, derivative_idx, value)

  std::vector<std::pair<double, double>> points;

  int radius = 1;
  int N = 10; // N
  double delta = 6.28 / N;

  double cos_d = cos(delta);
  double sin_d = sin(delta);

  double x = radius;
  double y = 0;  

  std::vector<p4::NodeEqualityBound> node_equality_bounds = {};

  for(int i = 0; i < N; i++){
    node_equality_bounds.push_back(p4::NodeEqualityBound(0,i,0,x));
    node_equality_bounds.push_back(p4::NodeEqualityBound(1,i,0,y));

    double new_x = x * cos_d - y * sin_d;
    double new_y = x * sin_d + y * cos_d;

    x = new_x;
    y = new_y;


    times.push_back(i);

  }

  node_equality_bounds.push_back(p4::NodeEqualityBound(0, N, 0, radius));
  node_equality_bounds.push_back(p4::NodeEqualityBound(1, N, 0, 0));

  times.push_back(N);

  // const std::vector<p4::NodeEqualityBound> node_equality_bounds = {
  //   ///////////////////////////////////////////////////////////////////
  //   // TODO: CREATE A CIRCLE TRAJECTORY
  //   ///////////////////////////////////////////////////////////////////

  // };

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

  // Sampling and Plotting
  { // Plot 2D position
    // Options to configure the polynomial sampler with
    p4::PolynomialSampler::Options sampler_options;
    sampler_options.frequency = 100;             // Number of samples per second
    sampler_options.derivative_order = 1;        // Derivative to sample (0 = pos)

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
}
