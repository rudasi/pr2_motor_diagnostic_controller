#include "pr2_motor_diagnostic_controller/diagnostic_controller.h" 
#include <pluginlib/class_list_macros.h>
#include <boost/thread/mutex.hpp>

PLUGINLIB_DECLARE_CLASS(pr2_diagnostic_controller, DiagnosticControllerPlugin, controller::Pr2DiagnosticController, pr2_controller_interface::Controller)

#define BUF_SIZE 4000

namespace controller
{

Pr2DiagnosticController::Pr2DiagnosticController()
: robot_(NULL), sample_buf_ptr_(NULL), sample_ptr_(NULL), buf_index_(0)
{
}

Pr2DiagnosticController::~Pr2DiagnosticController()
{
  delete[] sample_buf_ptr_;
}

bool Pr2DiagnosticController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  assert(robot);
  robot_ = robot;
  node_ = n;
  sample_buf_ptr_ = new ethercat_hardware::MotorTraceSample[BUF_SIZE];
  
  if(sample_buf_ptr_ == NULL)
  {
    ROS_ERROR("Could not create buffer for motor trace samples");
    return false;
  }
  std::string actuator_name;
  if (node_.getParam("/diag_actuator_name",actuator_name) == false)
  {
      ROS_ERROR("actuator name not published");
      return false;
  }
  sample_ptr_ = robot_->model_->hw_->getData<ethercat_hardware::MotorTraceSample>(actuator_name);
  if(sample_ptr_ == NULL)
  {
    ROS_ERROR("no data for actuator %s", actuator_name.c_str());
    return false;
  }
   
  srv_ = n.advertiseService("get_diagnostic_data",&Pr2DiagnosticController::getDiagnosticData, this);
  
  return true;
}

void Pr2DiagnosticController::update()
{
  if(robot_ == NULL)
  {
    ROS_ERROR("robot is null");
    return;
  }
  if(robot_->model_ == NULL)
  {
    ROS_ERROR("robot_->model_is null");
    return;
  }

  if(Pr2DiagnosticController::trylock())
  {
    ethercat_hardware::MotorTraceSample* s = &sample_buf_ptr_[buf_index_ % BUF_SIZE];

    s->timestamp = sample_ptr_->timestamp;
    s->enabled = sample_ptr_->enabled;
    s->supply_voltage = sample_ptr_->supply_voltage;
    s->measured_motor_voltage = sample_ptr_->measured_motor_voltage;
    s->programmed_pwm = sample_ptr_->programmed_pwm;
    s->executed_current = sample_ptr_->executed_current;
    s->measured_current = sample_ptr_->measured_current;
    s->velocity = sample_ptr_->velocity;
    s->encoder_position = sample_ptr_->encoder_position;
    s->encoder_error_count = sample_ptr_->encoder_error_count;

    buf_index_++;
    //unlock mutex
    Pr2DiagnosticController::unlock(); 
  }
  if(buf_index_ >= BUF_SIZE)
  {
    buf_full_ = true;
  }

}

void Pr2DiagnosticController::stopping()
{
}

void Pr2DiagnosticController::starting()
{
}

bool Pr2DiagnosticController::getDiagnosticData(pr2_motor_diagnostic_controller::DiagnosticData::Request& req, pr2_motor_diagnostic_controller::DiagnosticData::Response& resp)
{
  if(buf_full_)
  {
    //lock mutex, if not succesful will block on mutex
    //ok to lock here as buf_full_ is never reset
    Pr2DiagnosticController::lock();

    for(int i = 0; i < BUF_SIZE; i++)
    {
      pr2_motor_diagnostic_controller::MotorSample sample;
      sample.timestamp = sample_buf_ptr_[(i + buf_index_) % BUF_SIZE].timestamp;
      sample.enabled = sample_buf_ptr_[(i + buf_index_) % BUF_SIZE].enabled;
      sample.supply_voltage = sample_buf_ptr_[(i + buf_index_) % BUF_SIZE].supply_voltage;
      sample.measured_motor_voltage = sample_buf_ptr_[(i + buf_index_) % BUF_SIZE].measured_motor_voltage;
      sample.programmed_pwm = sample_buf_ptr_[(i + buf_index_) % BUF_SIZE].programmed_pwm;
      sample.executed_current = sample_buf_ptr_[(i + buf_index_) % BUF_SIZE].executed_current;
      sample.measured_current = sample_buf_ptr_[(i +  buf_index_) % BUF_SIZE].measured_current;
      sample.velocity = sample_buf_ptr_[(i + buf_index_) % BUF_SIZE].velocity;
      sample.encoder_position = sample_buf_ptr_[(i + buf_index_) % BUF_SIZE].encoder_position;
      sample.encoder_error_count = sample_buf_ptr_[(i + buf_index_) % BUF_SIZE].encoder_error_count;
      resp.sample_buffer.push_back(sample);
    }
    //unlock mutex
    Pr2DiagnosticController::unlock();
  }
  else
  {
    ROS_INFO("sample buffer is not yet full");
    return false;
  }
  return true;
}
 
}//end of namespace controller
