#include <cppad/cppad.hpp>
#include <mpc/NLMPC.hpp>
#include <mpc/Utils.hpp>
#include <math.h>
#include <cppad/ipopt/solve.hpp>
#include <cstddef>
#include "Kbicyclemodel.hpp" 

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"


#include "Eigen-3.3/Eigen/src/Core/Matrix.h"
#include"MPC_solver.hpp" 
#include <vector>
using namespace std;
using namespace Eigen;
using CppAD::AD; 

const int W_cross_track_e = 3000 ; 
 const int W_psi_err = 500 ; 
 const int W_vel_e = 1 ; 
 const int W_steer_ue = 1; 
 const int W_accel_ue = 1  ; 
 const int W_gap_delta = 200 ; 
 const int W_gap_accel = 1 ; 
   size_t steps = 15; 
 double v_ref = 40    ; 
 size_t N = 15 ; 


constexpr int Tnx = 4 ; // state vector A dimension
constexpr int Tnu = 2 ; // input dimesnion 
constexpr int Tny = 4 ; // output vector 
constexpr int Tph = 15 ; // prediction horizon 
constexpr int Tch = 15 ; //  control horizon 
constexpr int Tineq = 0   ; // inequality constraints no obstacles 
constexpr int Teq = 0 ; // equality constraints may be tph * state variable make sense 
double Ts = 0.15 ;  //discretization time 

mpc::NLMPC<Tnx, Tnu, Tny, Tph, Tch, Tineq, Teq> controller;
controller.setLoggerLevel(mpc::Logger::log_level::NORMAL);
// initializing the MPC controller with the paramters 

mpc::mat<Tnx, Tnx> A(Tnx, Tnx);
mpc::mat<Tnx, Tnu> B(Tnx, Tnu); // define the bicyle model here 
mpc::mat<Tny, Tnx> C(Tny, Tnx);
mpc::mat<Tny, Tnu> D(Tny, Tnu);

// implement the matrix A B C D 


// C is idenitty matrix 
// D is zero 

// call get the next state   
// generate the next state  here 
// give the address of the function into a lambda or a functor 
// to be given to the opitmizer  
// auto outEq =  car.generatenextstates 
 // function that get the output matrix 
controller.setOutputFunction(outEq); 
//
////////////OBJECTIVE J 
// call the function that return the cost as scalar 
// give it to controller.setobjectivefunct 
// calculate cost based on c , y , u , e,s 

// auto objEq = cost func that returns a scalar 
controller.setObjectiveFunction(objEq);



// inequality constraints sets , it not needed here 
// auto = an empty function needed only in case of obstacles 


controller.setIneqConFunction(conIneq); 
/// may remove this and add equality constraintss 

// set the intiial values here 
// call the init method 
// do the parameters 
mpc::NLParameters params;
params.maximum_iteration = 100;
params.relative_ftol = -1;
params.relative_xtol = -1;
params.hard_constraints = false;
params.enable_warm_start = true;
controller.setOptimizerParameters(params);
// assign the parameters 
auto res = controller.getLastResult();
res.cmd.setZero();  // initialize the result 
// initialized 
// start time  t =0  
double t =0 ; 
// do the for loop of iteration 


for(int i = 0; i < 150; i++)
{
    // solve
    res = controller.optimize(m_x, res.cmd);

 // m_x is the current state 
 // result .cmd contains the new control inpu retuned after the first iteration
// call the generate next state 


/*
    // apply vector field
    stateEq(m_x_next, m_x, res.cmd, -1);
    m_x = m_x_next;
    t += Ts;
*/

// update time 




// output the resultsssss

 double ref_cte = 0; // reference track
double ref_epsi = 0; // reference orientation 
const double Lf = 2.67 ; 





//initials atets are zeros 
//INDICES OF THE VECTOR
size_t x_start_index = 0  ;
size_t y_start_index  = x_start_index + steps;
size_t psi_start_index = y_start_index+steps;
size_t v_start_index = psi_start_index + steps ;
// errors  :
size_t cross_tracking_error_start_index  = v_start_index + steps;
size_t error_psi_start_index  = cross_tracking_error_start_index + steps;
size_t delta_input_start_index  = error_psi_start_index + steps;
size_t a_input_start_index = delta_input_start_index + steps -1 ; 
double dt = 0.15 ;  // discretization time taken from papers /tutorialss 
size_t Duration = 15  ; // number of timesteps prediction horizon = 0.15*15 = 2.25 seconds  
// it satisfies the constraints    p/20 <<disc_trime << p/10   p is prediction horizon . 

MPC::MPC( double discretization_time , size_t num_of_steps)  : dt(discretization_time), N_steps(num_of_steps),  car(KinematicBicycleModel::get_kinematic_bicycle_model())       {
 // initializing the mpc with the model , number of steps , duraion time       
 Lf = car.get_lenght_lf() ; 

} 
size_t MPC::get_control_horizon() {
    return control_horizon ; 
}
void MPC::set_control_horizon(size_t ch){
    control_horizon = ch ; 
}
double MPC::get_prediction_horizon(){
    return prediction_horizon ; 
}
void MPC::set_numberof_steps(size_t N_steps){
    this->N_steps = N_steps ; 
}
void MPC::set_time_step(double t){
    this->dt = t ; 
}
void MPC::set_ref_vel( double v){
    ref_vel = v ; 
}






class JCostFunction {
public : 
 Eigen::VectorXd poly_coeffs;
 // WEIGHTS OF THE ERRORS 
   size_t steps = 15; 
 double v_ref = 40    ; // check for error 
double v_reference = 40 ; 
  // Coefficients of the fitted polynomial.
JCostFunction(Eigen::VectorXd coeffecients , size_t N_steps ) :   steps(N_steps)  {
this->poly_coeffs = coeffecients ; 
// constructs the Jcost function 
}
typedef CPPAD_TESTVECTOR(AD<double>) ADvector;  // search for this 

// vars vector contains all of the states at all timesteps within the horizon 
/*

  var ==    [x0,x1,x2,x3,x4,x4---Xn , y0,y1,y2,----Yn , psi0,---ps_N , t ] 
  N duplicates of x,y,psi,v,cte,epsi,delta,acc



*/
void operator()(ADvector& cost_constraints_func , const ADvector& vars) {
cost_constraints_func[0] = 0 ;  // initial entry [0] is for the value of the cost function  
// J = summation ( all weight(i) * errors(i))
    cost_constraints_func[0] = 0;

    for (int t = 0; t < steps; t++) {  
      cost_constraints_func[0] += 3000*CppAD::pow(vars[cross_tracking_error_start_index + t], 2);
      cost_constraints_func[0] += 500*CppAD::pow(vars[error_psi_start_index + t], 2);
      cost_constraints_func[0] += CppAD::pow(vars[v_start_index + t] - v_ref, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < steps - 1; t++) { // number of inputs = N-1
      cost_constraints_func[0] += CppAD::pow(vars[delta_input_start_index + t], 2);
      cost_constraints_func[0] += CppAD::pow(vars[a_input_start_index + t], 2);
    }
    
    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < steps - 2; t++) { // control horizon = N-2
      cost_constraints_func[0] += 200 * CppAD::pow(vars[delta_input_start_index + t + 1] - vars[delta_input_start_index + t], 2);
      cost_constraints_func[0] += CppAD::pow(vars[a_input_start_index + t + 1] - vars[a_input_start_index + t], 2);
    }
// cost function  has now included all the square of erros and inputs misuse 
// add the constraints starting from position 1  
  // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    // map the state variable plus the errors cinstraints place 
    cost_constraints_func[1 + x_start_index] = vars[x_start_index];
    cost_constraints_func[1 + y_start_index] = vars[y_start_index];
    cost_constraints_func[1 + psi_start_index] = vars[psi_start_index];
    cost_constraints_func[1 + v_start_index] = vars[v_start_index];
    cost_constraints_func[1 + cross_tracking_error_start_index] = vars[cross_tracking_error_start_index];
    cost_constraints_func[1 + error_psi_start_index] = vars[error_psi_start_index];
// constraints to not deivate 


for (int t = 1; t < steps; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start_index + t];
      AD<double> y1 = vars[y_start_index + t];
      AD<double> psi1 = vars[psi_start_index + t];
      AD<double> v1 = vars[v_start_index + t];
      AD<double> cte1 = vars[cross_tracking_error_start_index + t];
      AD<double> epsi1 = vars[error_psi_start_index + t];

      // The state at time t.
      AD<double> x0 = vars[x_start_index + t - 1];
      AD<double> y0 = vars[y_start_index + t - 1];
      AD<double> psi0 = vars[psi_start_index + t - 1];
      AD<double> v0 = vars[v_start_index + t - 1];
      AD<double> cte0 = vars[cross_tracking_error_start_index + t - 1];
      AD<double> epsi0 = vars[error_psi_start_index + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_input_start_index + t - 1];
      AD<double> a0 = vars[a_input_start_index + t - 1];

      AD<double> f0 = poly_coeffs[0] + poly_coeffs[1] * x0 + poly_coeffs[2] * CppAD::pow(x0,2)+poly_coeffs[3] * CppAD::pow(x0,3);
      AD<double> psides0 = CppAD::atan(poly_coeffs[1]+2*poly_coeffs[2]*x0+3*poly_coeffs[3]*CppAD::pow(x0,2));
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      cost_constraints_func[1 + x_start_index + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt); // difference between predicted and model 
      cost_constraints_func[1 + y_start_index + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt); //MDC
      cost_constraints_func[1 + psi_start_index + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt); // generated by mpc esitimated - predicted by the model 
      cost_constraints_func[1 + v_start_index + t] = v1 - (v0 + a0 * dt);
      cost_constraints_func[1 + cross_tracking_error_start_index + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt)); // cross track error on the road width  
      cost_constraints_func[1 + error_psi_start_index + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt); 
     } // to avoid the local minimaaaaaa  
// cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt)

  }

} ; 
std::vector<double> MPC::MPC_Solve(Eigen::VectorXd initial_state_x0 , Eigen::VectorXd coeff ){
 const size_t N = 15 ;  
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = initial_state_x0[0];
  double y = initial_state_x0[1];
  double psi = initial_state_x0[2];
  double v = initial_state_x0[3];
  double cte = initial_state_x0[4];
  double epsi = initial_state_x0[5]; // states x y psi v cte eveler
std::cout<<" everything is good after initilaization \n " ; 


  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //  size_t i;
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  std::cout<< " \n  everything is good before for loop n vars \n " ; 
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
std::cout<< " \n  everything is good after for loop n vars \n " ; 
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_input_start_index; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (int i = delta_input_start_index; i < a_input_start_index; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_input_start_index; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }



  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start_index] = x;
  constraints_lowerbound[y_start_index] = y;
  constraints_lowerbound[psi_start_index] = psi;
  constraints_lowerbound[v_start_index] = v;
  constraints_lowerbound[cross_tracking_error_start_index] = cte;
  constraints_lowerbound[error_psi_start_index] = epsi;

  constraints_upperbound[x_start_index] = x;
  constraints_upperbound[y_start_index] = y;
  constraints_upperbound[psi_start_index] = psi;
  constraints_upperbound[v_start_index] = v;
  constraints_upperbound[cross_tracking_error_start_index] = cte;
  constraints_upperbound[error_psi_start_index] = epsi;

  // object that computes objective and constraints
  JCostFunction J_evaluator(coeff ,N) ; 
 //  FG_eval fg_eval(coeffs);

std::cout<< " \n  everything is before options \n " ; 


  
  // options for IPOPT solver
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  std::cout<< " \n  everything is good before \n " ; 
  CppAD::ipopt::solve<Dvector, JCostFunction>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, J_evaluator, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
std::cout<< " \n  everything is good after solve \n " ; 
  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;
  std::vector<double> res;
std::cout<< " \n  everything is good before pushback  \n " ; 
  res.push_back(solution.x[delta_input_start_index]);
  res.push_back(solution.x[a_input_start_index]);
std::cout<< " \n  everything is good after pushback \n " ; 
  for (int i = 0; i < N-1; i++) {
    res.push_back(solution.x[x_start_index + i + 1]);
    res.push_back(solution.x[y_start_index + i + 1]);
  }
std::cout<< " \n  everything is good before  return \n " ; 
  return res;



}

// IPOPT does accept a functor / lambda function , no support to functions so we shoud add our state vector in a class 
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


int main() {
  // MPC is initialized here!
  

  // Example telemetry data (You can replace these with actual test data)
  std::vector<double> ptsx = {10, 20, 30, 40};
  std::vector<double> ptsy = {10, 20, 30, 40};
  double px = 5;    // Current x position
  double py = 5;    // Current y position
  double psi = 0.1; // Current orientation
  double v = 10;    // Current velocity

  // Convert to vehicle coordinates
  int N = ptsx.size();
  Eigen::VectorXd x_vehicle(N);
  Eigen::VectorXd y_vehicle(N);
  for (int i = 0; i < N; i++) {
    double x = ptsx[i] - px;
    double y = ptsy[i] - py;
    x_vehicle[i] = x * cos(psi) + y * sin(psi);
    y_vehicle[i] = -x * sin(psi) + y * cos(psi);
  }

  // Fit polynomial
  auto coeffs = polyfit(x_vehicle, y_vehicle, 3);

  // Calculate cte and epsi
  double cte = coeffs[0];
  double epsi = -atan(coeffs[1]);

  // Define the state vector
  Eigen::VectorXd state(6);

  double dt = 0.1; // Time step
  double delta = 0.1; // Previous steering angle
  double prev_a = 0 ; // Previous throttle value
  double Lf = 2.67; // Distance between the front of the vehicle and its center of gravity
MPC mpc(dt,15) ; 
  // Predict state after latency
  double predicted_x = v * dt;
  double predicted_y = 0;
  double predicted_psi = -v * delta / Lf * dt;
  double predicted_v = v + prev_a * dt;
  double predicted_cte = cte + v * sin(epsi) * dt;
  double predicted_epsi = epsi + predicted_psi;

  state << predicted_x, predicted_y, predicted_psi, predicted_v, predicted_cte, predicted_epsi;
std::cout<< " \n  everything is good before cost \n " ; 
// problem in cost MPC_solve
  // Solve using MPC
  auto vars = mpc.MPC_Solve(state, coeffs);
std::cout<< " \n  everything is good after cost  \n " ; 
  // Extract the steering and throttle values
  double steer_value = vars[0];
  double throttle_value = vars[1];

  // Print the results
  std::cout << "Steering Value: " << steer_value << std::endl;
  std::cout << "Throttle Value: " << throttle_value << std::endl;

  // Print the predicted trajectory
  std::cout << "Predicted trajectory:" << std::endl;
  for (int i = 2; i < vars.size(); i += 2) {
    std::cout << "(" << vars[i] << ", " << vars[i + 1] << ")" << std::endl;
  }

  return 0;
}






















