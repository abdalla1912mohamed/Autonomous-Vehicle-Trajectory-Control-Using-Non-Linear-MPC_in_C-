#ifndef KINEMATIC_BICYCLE_MODEL_HPP
#define KINEMATIC_BICYCLE_MODEL_HPP


#include <math.h>
#include <iostream>
#include <utility>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
// class implementing the kinematic bicycle model using singleton pattern 


class KinematicBicycleModel {

public : 
// a static function returning the singleton instance 
static KinematicBicycleModel& get_kinematic_bicycle_model()  ; 

    // delete all copy/move constructors 
KinematicBicycleModel(KinematicBicycleModel const&) = delete;

void operator=(KinematicBicycleModel const&) = delete;

KinematicBicycleModel(KinematicBicycleModel&&) = delete;

void operator=(KinematicBicycleModel&&) = delete;

// void discretize_Model() ; 

  
void set_states_size(int s) ; 
int get_state_size() ; 
  
double get_lenght_lf() ; 


Eigen::VectorXd generate_next_state(  Eigen::VectorXd state_inputs , double dt)  ; 

 std::pair<double,double> get_xycoordinates() ; 
 double get_orientation_d() ; 
  double get_orientation_r() ; 
 double get_velocity() ; 

void set_length_c2f(double lf ) ; 
void set_states(double x , double y , double psi , double v) ; 
void set_states(Eigen::VectorXd states_to_set);

Eigen::VectorXd get_states() ; 
private : 
KinematicBicycleModel() ; 
 double Lf ; // length from center  to front
Eigen::VectorXd states ; // an array of internal states
int states_size = 4 ; 

// states X are [ x1  x2  x3  x4] where x1-> x coordinate , x2-> y-coordinate , x3-> psi angle (orientation ), x4->velocity v 
// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt   // discretized model 
// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
// v_[t+1] = v[t] + a[t] * dt 
// inputs are U = [ u1 , u2] , u1-> steering angle &  and u2 -> a acceleration 
} ; 

#endif