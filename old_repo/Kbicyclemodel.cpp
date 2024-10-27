#include <cmath>
#include <iostream> 
#include"Kbicyclemodel.hpp" 


// #include "Eigen-3.3/Eigen/src/Core/Matrix.h"
// by Abdalla Mohamed    
// the cpp driver for the kienamtic car model  --- 

double degree_to_radian(double ceta){

return ceta*M_PI/180 ; 

}
double radian_to_degree(double radian){

return radian*180/M_PI ; 

}
// function to return the only available instance of the bicycle car model  ; 
KinematicBicycleModel & KinematicBicycleModel::get_kinematic_bicycle_model(){
    static KinematicBicycleModel  KBM_car ; 
    
    return KBM_car ; 
}
KinematicBicycleModel::KinematicBicycleModel() : Lf(2.0), states(states_size){
    
}

std::pair<double,double> KinematicBicycleModel::get_xycoordinates(){
auto p = std::make_pair(states[0],states[1]) ;  
return p ; 
}

double KinematicBicycleModel::get_orientation_d(){

return states[2] ; 
}

double KinematicBicycleModel::get_velocity(){
    return states[3] ; 
}


double KinematicBicycleModel::get_orientation_r(){
return degree_to_radian(states[2]) ; 
}
double KinematicBicycleModel::get_lenght_lf(){
    return this->Lf ; 
}



void KinematicBicycleModel::set_states_size(int s) {
    this->states_size = s ; 
} 
int KinematicBicycleModel::get_state_size(){
    return this->states_size ; 
}
Eigen::VectorXd KinematicBicycleModel::get_states(){
    return states ; 
}


void KinematicBicycleModel::set_length_c2f(double lf ) {
    this->Lf=lf ; 
}
void KinematicBicycleModel::set_states(double x, double y, double psi, double v )  {
    // Ensure states vector is correctly sized
    set_states_size(4) ; 
    states[0] = x;
    states[1] = y;
    states[2] = psi;
    states[3] = v;
}


void KinematicBicycleModel::set_states(Eigen::VectorXd states_to_set) {
    // Ensure the input vector has the correct size
    if (states_to_set.size() == states_size) {
        states = states_to_set;
    } else {
        std::cerr << "Error: Input state vector has incorrect size." << std::endl;
        // Handle the error as needed
    }
}

Eigen::VectorXd KinematicBicycleModel::generate_next_state(  Eigen::VectorXd state_inputs , double dt) {
 
 Eigen::VectorXd next_state(this->states_size);

  auto x = this->states(0);       
  auto y = this->states(1);
  auto psi = this->states(2);
  auto v = this->states(3);
  auto delta = state_inputs(0);
  auto a = state_inputs(1);


  // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
  this->states(0) = x + v * cos(psi) * dt;

  // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
  this->states(1) = y + v * sin(psi) * dt;
  // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
  this->states(2) = psi + v / Lf * delta * dt;
  // v_[t+1] = v[t] + a[t] * dt
  this->states(3) = v + a * dt;
  

  return states;

}

static KinematicBicycleModel& Kinematic_Model_INIT(Eigen::VectorXd initial_state , int state_size , int Lf_length){
KinematicBicycleModel& car =  KinematicBicycleModel::get_kinematic_bicycle_model() ; 
   car.set_length_c2f(Lf_length) ; 
   car.set_states_size(state_size) ; 
   car.set_states(initial_state) ; 
   
   return car ; 
}
/*
int main(){
    
  //KinematicBicycleModel& car =  KinematicBicycleModel::get_kinematic_bicycle_model() ; 
   //car.set_length_c2f(2.00) ; 
   //car.set_states_size(4) ; 

   Eigen::VectorXd input (2) ; 
   Eigen::VectorXd initial(4) ; 

   initial << 0, 0,degree_to_radian(45),1 ; 
   input << degree_to_radian(5), 1;
   std::cout<<initial<<std::endl  ; 
auto & e = Kinematic_Model_INIT(initial, 4, 2) ; 


   std::cout<<e.generate_next_state(input, 0.3)<<std::endl ; 

    
}

*/
