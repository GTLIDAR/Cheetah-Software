#ifndef A1JPOS_CONTROLLER
#define A1JPOS_CONTROLLER

#include <RobotController.h>
#include "A1JPosUserParameters.h"

class A1JPos_Controller:public RobotController{
  public:
    A1JPos_Controller():RobotController(),_jpos_ini(cheetah::num_act_joint){
    _jpos_ini.setZero();
    }
    virtual ~A1JPos_Controller(){}

    virtual void initializeController(){}
    virtual void runController();
    virtual void updateVisualization(){}
    virtual ControlParameters* getUserControlParameters() {
      return &userParameters;
    }
  protected:
    DVec<float> _jpos_ini;
  A1JPosUserParameters userParameters;
};

#endif
