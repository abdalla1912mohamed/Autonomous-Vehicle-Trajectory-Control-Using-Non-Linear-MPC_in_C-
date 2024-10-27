#include <mpc/NLMPC.hpp>
#include <mpc/Utils.hpp>
#include <fstream>
#include <vector>
//#include "matplotlibcpp.h"
//namespace plt = matplotlibcpp;

// Constants for the car model
constexpr double wheelbase = 2.0;  // Example wheelbase length
constexpr double v_ref = 50.0;      // Constant reference velocity (e.g., 1 m/s)
constexpr double Ts = 0.1;  
       // Sampling time
const int iterations = 9980; 
std::vector<double> vrefs(iterations) ; 
std::vector<double> yrefs(iterations) ;
double psi_ref = 0 ; 
std::vector<std::vector<int>> vec_plot(12, std::vector<int>(iterations));

// vector to save x_ref,y_ref,x_real,y_real,psi_ref,psi_real,v_ref,v_real,cost,time , input1 , input2 
static int indexer  = 0 ; 
double x_ref = 0 ; 

int main()
{
    //plt::plot({1,3,2,4});
    //plt::show();

         
    for(int i = 0 ; i<iterations ; i++){
    if(i<=500){
        vrefs[i] = 5; 
        yrefs[i] = 1 ; 

    }
if(i>500  && i<=2000 )
vrefs[i] = 40 ; 
yrefs[i] = 3 ; 

if(i>2000 && i<=4000){
    vrefs[i] = 20  ;
    yrefs[i]  = 2 ;  
}
if(i>4000  && i<6000){
    vrefs[i] = 3  ; 
     yrefs[i]  = -2 ;  
} else {
    vrefs[i] = 5  ; 
     yrefs[i]  = 0 ;  

}





}
    // State vector: [x, y, theta, v]
    constexpr int Tnx = 4; 

    // Control vector: [a, delta] (acceleration, steering angle)
    constexpr int Tnu = 2;

    // Output vector: [x, y, theta, v]
    constexpr int Tny = 4;

    // Prediction and control horizon lengths
    constexpr int Tph = 10;
    constexpr int Tch = 10;

    // No equality constraints, but we might have inequality constraints (optional)
    constexpr int Tineq = 0;
    constexpr int Teq = 0;

    // Create the MPC controller
    mpc::NLMPC<Tnx, Tnu, Tny, Tph, Tch, Tineq, Teq> controller;
    controller.setLoggerLevel(mpc::Logger::log_level::NORMAL);

    // Set up the state equation for the kinematic car model
    auto stateEq = [&](
                       mpc::cvec<Tnx> &dx,
                       const mpc::cvec<Tnx> &x,
                       const mpc::cvec<Tnu> &u,
                       const unsigned int &)
    {
        // x = [x_pos, y_pos, theta, v], u = [a, delta]
        dx(0) = x(0)+ (x(3) * cos(x(2)))*Ts;       // x_dot = v * cos(theta)
        dx(1) = x(1) + (x(3) * sin(x(2)))*Ts;       // y_dot = v * sin(theta)
        dx(2) = x(2) + ((x(3) / wheelbase) * tan(u(1)))*Ts;  // theta_dot = (v / L) * tan(delta)
        dx(3) = x(3)+ u(0)*Ts;   
        
        
/*
// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
  this->states(0) = x + v * cos(psi) * dt;

  // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
  this->states(1) = y + v * sin(psi) * dt;
  // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
  this->states(2) = psi + v / Lf * delta * dt;
  // v_[t+1] = v[t] + a[t] * dt
  this->states(3) = v + a * dt;*/







        
                        // v_dot = acceleration (a)
    };
    controller.setStateSpaceFunction(stateEq);

    // Set up the output equation (can be an identity map in this case)
    auto outEq = [&](
                     mpc::cvec<Tny> &y,
                     const mpc::cvec<Tnx> &x,
                     const mpc::cvec<Tnu> &u,
                     const unsigned int &)
    {
        y = x;  // Output is the same as the state vector
    };
    controller.setOutputFunction(outEq);

    // Define the objective function (minimizing velocity error and control effort)
    auto objEq = [&](
                     const mpc::mat<Tph + 1, Tnx> &x,
                     const mpc::mat<Tph + 1, Tny> &y,
                     const mpc::mat<Tph + 1, Tnu> &u,
                     const double &e  )
    {
        double cost = 0;
        for (int i = 0; i < Tph + 1; i++)
        {
            
            // Penalize deviation from reference velocity
            cost += 15*(x(i, 3) - vrefs[i+indexer]) * (x(i, 3) - vrefs[i+indexer]); 

            // Penalize control effort (acceleration and steering angle)
            cost += 100 * u(i,0)*u(i,0);
            cost += 300 * u(i,1)*u(i,1);

            if(i<Tph-1){
            cost+= 10 * (u(i+1,0)-u(i,0))* (u(i+1,0)-u(i,0));  // weight  on changing a
            cost+= 100 * (u(i+1,1)-u(i,1))* (u(i+1,1)-u(i,1)); // weight on changing delta  // it was 500 
}
            cost+= 1000*( (yrefs[i+indexer]-x(i,1))*(yrefs[i+indexer]-x(i,1))   )     ; 
            
             // cross track error   y local                                    // cross track error  cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt)
            // difference in u 
        }
         
        // Optional: penalize slack variables if soft constraints are used
       // cost += 1e-5 * e * e;

        return cost;
    };
    controller.setObjectiveFunction(objEq);

    // Set up the MPC parameters
    mpc::NLParameters params;
    params.maximum_iteration = 100;
    params.relative_ftol = -1;
    params.relative_xtol = -1;
    params.hard_constraints = false;
    params.enable_warm_start = true;

    controller.setOptimizerParameters(params);

/**/    // State vector bounds

mpc::cvec<Tnx> xmin, xmax;
xmin << -1000000000, -5, -mpc::inf, 0;  // [x_min, y_min, theta_min, v_min]
xmax <<  1000000000, 5, mpc::inf, 80;   // [x_max, y_max, theta_max, v_max]

mpc::cvec<Tnu> umin, umax;
umin << -3, -0.5; // Minimum bounds for control inputs
umax << 3, 0.5;   // Maximum bounds for control inputs

mpc::HorizonSlice slice = {0, Tch}; // Apply bounds to the entire control horizon

bool result = controller.setInputBounds(umin, umax, slice);
if (!result) {
    std::cerr << "Failed to set input bounds" << std::endl;
}



 controller.setStateBounds(xmin, xmax, {0, Tph});


    // Initialize the state and control input
    mpc::cvec<Tnx> m_x, m_x_next;
    m_x.setZero();  // Initial state [x=0, y=0, theta=0, v=0]
     m_x(0) = 0 ;  // initilaize the state to zero 
     m_x(1)= 0  ; 
     m_x(2) = 0 ; 
    auto res = controller.getLastResult();
    res.cmd.setZero();  // Initial control input [a=0, delta=0]

    double t = 0;
    for (int i = 0; i < iterations; i++)
    {
        // Solve the MPC problem
        res = controller.optimize(m_x, res.cmd);
        // the input 
        // Apply the control input to the state equation
        stateEq(m_x_next, m_x, res.cmd, -1);
        m_x = m_x_next;  // Update the state // intergate 
        t += Ts;

        // Print the state and control information
        std::cout<<" iteration ::  "<<i  << std::endl ; 
        std::cout << t << "= total time  , X = " << m_x(0) << " Y = " << m_x(1) << " ,PSI = " << m_x(2) << " ,  V =" << m_x(3)<<std::endl;
        std::cout <<"vref :: "<< vrefs[i]<<std::endl ; 
        std::cout << "input 1 =  " << res.cmd(0) << "  ,input 2 =" << res.cmd(1) << std::endl;
        std::cout << " cost = ," << res.cost << " solver status = ," << res.solver_status;
        std::cout << std::endl;
        
        vec_plot[0][i]=x_ref ; 
        vec_plot[1][i]=yrefs[i] ; 
        vec_plot[2][i]=m_x(0) ; 
        vec_plot[3][i]=m_x(1) ; 
        vec_plot[4][i]=psi_ref ;
        vec_plot[5][i]=m_x(2) ; 
        vec_plot[6][i]=vrefs[i] ; 
        vec_plot[7][i]=m_x(3) ; 
        vec_plot[8][i]=res.cost ; 
        vec_plot[9][i]=t;  // time 
        vec_plot[10][i]=res.cmd(0) ; 
        vec_plot[11][i]=res.cmd(1);


indexer++ ;  
    }

    std::ofstream file("MPC_CHANGING_LANES_VARYING_VELOCITY.csv");
    if (!file) {
        std::cerr << "Unable to open file for writing." << std::endl;
        return 1;
    }

    // Write the matrix to the file
    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j < 10000; ++j) {
            file << vec_plot[i][j];
            if (j < 10000 - 1) {
                file << ",";
            }
        }
        file << "\n";
    }

    file.close();

    // Optional: Print execution stats
    // std::cout << controller.getExecutionStats();

    return 0;
}
