#ifndef PR2_DIAGNOSTIC_CONTROLLER_H
#define PR2_DIAGNOSTIC_CONTROLLER_H

#include <pr2_controller_interface/controller.h>
#include <ros/ros.h>
#include <pr2_motor_diagnostic_controller/DiagnosticData.h>
#include <pr2_hardware_interface/hardware_interface.h>
#include <ethercat_hardware/MotorTraceSample.h>
#include <boost/thread/mutex.hpp>

namespace controller
{

class Pr2DiagnosticController: public pr2_controller_interface::Controller, boost::noncopyable
{
public:
  Pr2DiagnosticController();
  ~Pr2DiagnosticController();
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  virtual void starting();
  virtual void update();
  virtual void stopping();
  bool getDiagnosticData(pr2_motor_diagnostic_controller::DiagnosticData::Request& req, pr2_motor_diagnostic_controller::DiagnosticData::Response& resp);

private:
  pr2_mechanism_model::RobotState* robot_;
  ros::ServiceServer srv_;
  ros::NodeHandle node_;
  ethercat_hardware::MotorTraceSample *sample_buf_ptr_;
  ethercat_hardware::MotorTraceSample *sample_ptr_;
  int buf_index_;
  bool buf_full_;
  boost::mutex sample_mutex_;
 
  bool trylock()
  {
    if (sample_mutex_.try_lock())
    {
      return true;
    }
    else
    {
      return false;
    }
  }
 
  void lock()
  {
    sample_mutex_.lock();
  }

  void unlock()
  {
    sample_mutex_.unlock();
  }

  
};

}//end of controller namespace
#endif
