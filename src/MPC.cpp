#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

//Set the timestep length and duration
size_t N = 10; //number of steps
double dt = 0.1; //time increment

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

double ref_v = 40; //referance velocity


//variables to hold the index at which each state starts in the vector passed to the MPC
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
    // Fitted polynomial coefficients
	Eigen::VectorXd coeffs;
	// Coefficients of the fitted polynomial.
	FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	// `fg` is a vector containing the cost and constraints.
	// `vars` is a vector containing the variable values (state & actuators).
	void operator()(ADvector& fg, const ADvector& vars) {
		// The cost is stored is the first element of `fg`.
		// Any additions to the cost should be added to `fg[0]`.
		fg[0] = 0;

		// defining weights for the cost function
		double w_cte = 15;
		double w_epsi = 5;
		double w_v = 1;
		double w_delta = 50;
		double w_a = 5;
		double w_delta_variation = 30000;
		double w_a_variation = 200;

		// calculating the cost
		for (size_t itr = 0; itr < N; itr++)
		{
			// errors cost
			fg[0] += w_cte * CppAD::pow(vars[cte_start + itr], 2);
			fg[0] += w_epsi * CppAD::pow(vars[epsi_start + itr], 2);
			fg[0] += w_v * CppAD::pow(vars[v_start + itr] - ref_v, 2);


			// actuator effort cost
			if (itr < (N - 1))
			{
				fg[0] += w_delta * vars[v_start + itr] * CppAD::pow(vars[delta_start + itr], 2);
				fg[0] += w_a * CppAD::pow(vars[a_start + itr], 2);


				// actuator variations cost
				if (itr > 0) 
				{
					fg[0] += w_delta_variation * CppAD::pow(vars[delta_start + itr] - vars[delta_start + itr - 1], 2);
					fg[0] += w_a_variation * CppAD::pow(vars[a_start + itr] - vars[a_start + itr - 1], 2);
				}
			}
		}


		//
		// Setup Constraints
		//
		// setting up the model constraints.

		// Initial constraints
		//
		// We add 1 to each of the starting indices due to cost being located at
		// index 0 of `fg`.
		// This bumps up the position of all the other values.
		fg[1 + x_start] = vars[x_start];
		fg[1 + y_start] = vars[y_start];
		fg[1 + psi_start] = vars[psi_start];
		fg[1 + v_start] = vars[v_start];
		fg[1 + cte_start] = vars[cte_start];
		fg[1 + epsi_start] = vars[epsi_start];
		
		//Adding constraints on the initial actuation values assuming 
		//they would be the same for the first step of the controller 
		// to count for latency
		fg[1 + delta_start] = vars[delta_start];
		fg[2 + delta_start] = vars[a_start];

		// The rest of the constraints
		for ( int t = 1; t < N; t++) {
			// The state at time t+1 .
			AD<double> x1 = vars[x_start + t];
			AD<double> y1 = vars[y_start + t];
			AD<double> psi1 = vars[psi_start + t];
			AD<double> v1 = vars[v_start + t];
			AD<double> cte1 = vars[cte_start + t];
			AD<double> epsi1 = vars[epsi_start + t];

			// The state at time t.
			AD<double> x0 = vars[x_start + t - 1];
			AD<double> y0 = vars[y_start + t - 1];
			AD<double> psi0 = vars[psi_start + t - 1];
			AD<double> v0 = vars[v_start + t - 1];
			AD<double> cte0 = vars[cte_start + t - 1];
			AD<double> epsi0 = vars[epsi_start + t - 1];
			
			// actuation values at time t
			AD<double> delta0 = vars[delta_start + t - 1];
			AD<double> a0 = vars[a_start + t - 1];
			

			//find desired y position based on fitted polynomial
			AD<double> f0 = 0;
			for ( int j = 0; j < coeffs.size(); j++)
			{
				f0 += coeffs[j] * CppAD::pow(x0, j);
			}

			//find desired orientation based on fitted polynomial
			AD<double> f1 = 0;
			for ( int j = 1; j < coeffs.size(); j++)
			{
				f1 += j * coeffs[j] * CppAD::pow(x0, j-1);
			}
			AD<double> psides0 = CppAD::atan(f1);

			//Constraints related to the update of the states
			//These constraints should be zero
			//
			// The equations for the model:
			// x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
			// y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
			// psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
			// v_[t] = v[t-1] + a[t-1] * dt
			// cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
			// epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
			fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
			fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
			fg[1 + cte_start + t] =
				cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
			fg[1 + epsi_start + t] =
				epsi1 - ((psides0 - psi0) + v0 * delta0 / Lf * dt);
		}
	}
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  double delta = state[6];
  double a = state[7];

  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N * 6 + (N-1)*2;
  // Number of constraints
  // Each one of the six kinematic states would have N contraints for each step
  // And an addition of two contraints for the initial steering and throttle values assuming latency
  size_t n_constraints = N * 6 + 2; 

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for ( int i = 0; i < delta_start; i++) {
	  vars_lowerbound[i] = -1.0e19;
	  vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for ( int i = delta_start; i < a_start; i++) {
	  vars_lowerbound[i] = -0.436332;
	  vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  for ( int i = a_start; i < n_vars; i++) {
	  vars_lowerbound[i] = -1.0;
	  vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for ( int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
  //The two additional contraints related to the 
  //initial actuation latency
  constraints_lowerbound[delta_start] = delta;
  constraints_lowerbound[delta_start + 1] = a;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;
  //The two additional contraints related to the 
  //initial actuation latency
  constraints_upperbound[delta_start] = delta;
  constraints_upperbound[delta_start + 1] = a;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);


  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  //Updating predicted x_vals and y_vals vector based on the new optimized actuations of the MPC

  MPC::x_vals.clear();
  MPC::y_vals.clear();
  //iterate over points
  for ( int j = 0; j < N; j++)
  {
	  MPC::x_vals.push_back(solution.x[x_start + j]);
	  MPC::y_vals.push_back(solution.x[y_start + j]);
  }
  

  // Return the vectors for the updated solutions
  // Returning the second optimized actuations as the first actuations were
  // assumed to remain the same due to latency
  return {solution.x[delta_start+1], solution.x[a_start+1]};
}
