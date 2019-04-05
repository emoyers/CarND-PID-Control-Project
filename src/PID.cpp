#include "PID.h"

#define evaluation_counts 850

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;

  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;

  this->count = 0;
  this->total_error = 0;
  this->best_error = std::numeric_limits<double>::max();
  this->error_initilialize = false;
  this->reach_cruise_speed = false;
  this->new_best_error = false;

  this->previous_add = false;
  this->previous_sub = false;

  this->dp = {0.1*this->Kp,0.1*this->Ki,0.1*this->Kd};
  this->index_dp = 0;


}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  this->d_error = cte - this->p_error;
  this->p_error = cte;
  this->i_error += cte;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double result = -((this->Kp*this->p_error)+(this->Kd*this->d_error)+(this->Ki*this->i_error));
  return  result;// TODO: Add your total error calc here!
}

void PID::Twiddle(double cte){

  if(this->count < evaluation_counts ){

    this->total_error += pow(cte,2);

    if(this->reach_cruise_speed == false){
      this->reach_cruise_speed = true;
    }

  }
  else if(this->error_initilialize != true){
    this->best_error = this->total_error;
    this->error_initilialize = true;
    this->count = 0;
    this->total_error = 0;
    cout<<"first time error: "<<this->best_error<<endl;
  }
  else{

    if(!(this->previous_add) && !(this->previous_sub)){
      PID::modify_pid_parameters(this->index_dp, this->dp[this->index_dp]);
      this->previous_add = true;
    }
    else{

      if(this->total_error<this->best_error){
        cout<<"previous_best_error: "<<this->best_error<<endl;
        this->best_error = this->total_error;
        this->dp[this->index_dp] *= 1.1; 
        this->new_best_error = true;
        this->previous_add = false;
        this->previous_sub = false;
        this->index_dp = (this->index_dp+1)%3;
        cout<<"best_error: "<<this->best_error<<endl;
        cout<<"----------------------new values------------------------------"<<endl;
        cout<<"KP: "<<this->Kp<<", KI: "<<this->Ki<<", KD: "<<this->Kd<<endl;
        cout<<"----------------------new values------------------------------"<<endl;
        cout << "CTE: " << cte << endl;
      }
      else{
        this->new_best_error = false;
      }

      if(!(this->new_best_error) && (this->previous_add) && !(this->previous_sub)){
        PID::modify_pid_parameters(this->index_dp, -2*this->dp[this->index_dp]);
        this->previous_sub = true;
      }
      else if (!(this->new_best_error) && (this->previous_add) && (this->previous_sub)){
        PID::modify_pid_parameters(this->index_dp, this->dp[this->index_dp]);
        this->dp[this->index_dp] *= 0.9; 
        this->previous_add = false;
        this->previous_sub = false;
        this->index_dp = (this->index_dp+1)%3;
      }

    }

    this->count = 0;
    this->total_error = 0;
    cout<<"previous_add"<<this->previous_add<<endl;
    cout<<"previous_sub"<<this->previous_sub<<endl;
    cout<<"current values"<<endl;
    cout<<"KP: "<<this->Kp<<", KI: "<<this->Ki<<", KD: "<<this->Kd<<endl;


  }
}

void PID::modify_pid_parameters(int index, double value){
  if(index == 0){
    this->Kp += value;
  }
  else if(index == 1){
    this->Ki += value;
  }
  else{
    this->Kd += value;
  }
}