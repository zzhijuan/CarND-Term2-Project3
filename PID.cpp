#include "PID.h"
#include <iostream>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
   
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
    p_error = 0.0;
    d_error = 0.0;
    i_error = 0.0;
    
    //initialize twiddling parameters
    step = 1;
    option = 2;
    dp = {0.1*Kp,0.1*Kd,0.1*Ki};
    converg_step = 50;
    n_step = 600;
    total_error = 0;
    best_error = 1000000;
    adding = false;
    subtracting = false;

}

void PID::UpdateError(double cte) {
    if(step == 1)
    {
        p_error = cte;
    }
    
    d_error = cte - p_error;
    p_error = cte;
    i_error = cte + i_error;
    
    //update total error after n times converg step
    if(step%(converg_step + n_step) > converg_step)
    {
        total_error += pow(cte, 2);
    }
    
    //twiddle
    if(step%(converg_step + n_step) == 0)
    {
        if(total_error < best_error)
        {
            best_error = total_error;
            dp[option] *= 1.1;
            option = (option + 1)%3;
            adding = false;
            subtracting = false;
        }
        
        //adding
        if(!adding && !subtracting)
        {
            if(option == 0) Kp = Kp + dp[option];
            if(option == 1) Ki = Ki + dp[option];
            if(option == 2) Kd = Kd + dp[option];
            adding = true;
        }
        
        //subtracting
        else if(adding && !subtracting)
        {
            if(option == 0) Kp = Kp - 2 * dp[option];
            if(option == 1) Ki = Ki - 2 * dp[option];
            if(option == 2) Kd = Kd - 2 * dp[option];
            subtracting = true;
        }
        
        //reset, move to the next parameter
        else {
            if(option == 0) Kp = Kp + dp[option];
            if(option == 1) Ki = Ki + dp[option];
            if(option == 2) Kd = Kd + dp[option];
            dp[option] *= 0.9;
            option = (option + 1) % 3;
            adding = false;
            subtracting = false;

        }
        total_error = 0;
    }
    step = step + 1;
    
}

double PID::TotalError() {
    return 0.0;
}

