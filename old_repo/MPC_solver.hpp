#ifndef  MPC_solver_hpp 
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <cstddef>
#include "Kbicyclemodel.hpp" 
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/src/Core/Matrix.h"
#include <vector>


class MPC      {
public:


MPC( double discretization_time , size_t number_of_steps); 

MPC() = delete ; 

std::vector<double> MPC_Solve(Eigen::VectorXd initial_state_x0, Eigen::VectorXd coff);
   
void set_numberof_steps(size_t N_steps);
void set_time_step(double t);
void set_ref_vel( double v);
size_t get_control_horizon() ;
void set_control_horizon(size_t ch); 
double get_prediction_horizon(); 
// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
//states : 


private : 
KinematicBicycleModel & car  ;  
// We set the number of timesteps to 25
// and the timestep evaluation frequency or evaluation
// period to 0.05.
size_t N_steps    ; //  // number of time steps
double dt ;  // discretization time
double ref_vel  = 40   ; 
double prediction_horizon = N_steps*dt ; 
size_t control_horizon = N_steps - 2 ;  

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
 double Lf = 2.67;

// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 40 mph.


} ; 



















#endif // ! MPC_solver_hpp 


















