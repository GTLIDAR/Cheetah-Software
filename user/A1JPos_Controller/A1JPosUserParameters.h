#ifndef PROJECT_A1JPOSUSERPARAMETERS_H
#define PROJECT_A1JPOSUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class A1JPosUserParameters : public ControlParameters {
public:
  A1JPosUserParameters()
      : ControlParameters("user-parameters"),
        INIT_PARAMETER(tau_ff),
        INIT_PARAMETER(kp),
        INIT_PARAMETER(kd)
      {}

  DECLARE_PARAMETER(double, tau_ff);
  DECLARE_PARAMETER(double, kp);
  DECLARE_PARAMETER(double, kd);
};

#endif //PROJECT_JPOSUSERPARAMETERS_H
