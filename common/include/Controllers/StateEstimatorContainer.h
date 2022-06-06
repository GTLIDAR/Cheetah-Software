/*!
 * @file StateEstimator.h
 * @brief Implementation of State Estimator Interface
 *
 * Each StateEstimator object contains a number of estimators
 *
 * When the state estimator is run, it runs all estimators.
 */

#ifndef PROJECT_STATEESTIMATOR_H
#define PROJECT_STATEESTIMATOR_H

#include "ControlParameters/RobotParameters.h"
#include "Controllers/LegController.h"
#include "Controllers/GaitScheduler.h"
#include "Dynamics/Quadruped.h"
#include "SimUtilities/IMUTypes.h"
#include "SimUtilities/VisualizationData.h"
#include "state_estimator_lcmt.hpp"

/*!
 * Result of state estimation
 */
template <typename T>
struct StateEstimate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec4<T> contactEstimate;
  Vec3<T> position;
  Vec3<T> vBody;
  Quat<T> orientation;
  Vec3<T> omegaBody;
  RotMat<T> rBody;
  Vec3<T> rpy;

  Vec3<T> omegaWorld;
  Vec3<T> vWorld;
  Vec3<T> aBody, aWorld;

  void setLcm(state_estimator_lcmt& lcm_data) {
    for(int i = 0; i < 3; i++) {
      lcm_data.p[i] = position[i];
      lcm_data.vWorld[i] = vWorld[i];
      lcm_data.vBody[i] = vBody[i];
      lcm_data.rpy[i] = rpy[i];
      lcm_data.omegaBody[i] = omegaBody[i];
      lcm_data.omegaWorld[i] = omegaWorld[i];
    }

    for(int i = 0; i < 4; i++) {
      lcm_data.quat[i] = orientation[i];
    }
  }
};

/*!
 * Inputs for state estimation.
 * If robot code needs to inform the state estimator of something,
 * it should be added here. (You should also a setter method to
 * StateEstimatorContainer)
 */
template <typename T>
struct StateEstimatorData {
  StateEstimate<T>* result;  // where to write the output to
  Vec4<T>* contactPhase;
  Vec4<T> footHeights;
  Vec12<T> contactForces;
  LegControllerData<T>* legControllerData;

  const VectorNavData* vectorNavData;
  const CheaterState<double>* cheaterState;
  const Quadruped<T>* robot;
  const RobotControlParameters* parameters;
};

/*!
 * All Estimators should inherit from this class
 */
template <typename T>
class GenericEstimator {
 public:
  virtual void run() = 0;
  virtual void setup() = 0;

  void setData(StateEstimatorData<T> data) { _stateEstimatorData = data; }

  virtual ~GenericEstimator() = default;
  StateEstimatorData<T> _stateEstimatorData;
};

/*!
 * Main State Estimator Class
 * Contains all GenericEstimators, and can run them
 * Also updates visualizations
 */
template <typename T>
class StateEstimatorContainer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Construct a new state estimator container
   */
  StateEstimatorContainer(CheaterState<double>* cheaterState,
                          VectorNavData* vectorNavData,
                          LegControllerData<T>* legControllerData,
                          StateEstimate<T>* stateEstimate,
                          RobotControlParameters* parameters) {
    _data.cheaterState = cheaterState;
    _data.vectorNavData = vectorNavData;
    _data.legControllerData = legControllerData;
    _data.result = stateEstimate;
    _phase = Vec4<T>::Zero();
    _data.contactPhase = &_phase;
    _data.parameters = parameters;
  }

  /*!
   * Run all estimators
   */
  void run(CheetahVisualization* visualization = nullptr) {
    for (auto estimator : _estimators) {
      estimator->run();
    }
    if (visualization) {
      visualization->quat = _data.result->orientation.template cast<float>();
      visualization->p = _data.result->position.template cast<float>();
      // todo contact!
    }
  }

  /*!
   * Get the result
   */
  const StateEstimate<T>& getResult() { return *_data.result; }
  StateEstimate<T> * getResultHandle() { return _data.result; }

  /**
  * Position of the feet in the true world frame
  */
  Vec3<T> getFootPosWorld(int foot) {
      return _data.result.position +
             (_data.result.rBody.transpose()
              * (_robot->getHipLocation(foot)
                 + _data.legControllerData[foot]->p));
  }


  /**
   * Position of the feet in the world frame with the
   * CoM ground projection being the origin
   */
  Vec3<T> getFootPosCoMOrigin(int foot) {
      Vec3<T> pz(0.0, 0.0, _data.result.position(2));
      return pz + _data.result.rBody.transpose()
                  * (_robot->getHipLocation(foot)
                     + _data.legControllerData[foot]->p);
  }

  Vec3<T> getLocalFootPos(int foot){
      Vec3<T> pz(0., 0., _data.result.position(2));
      return pz + (_robot->getHipLocation(foot) + _data.legControllerData[foot]->p);
  }


  /**
   * Vector from the CoM to the foot in the body frame
   */
  Vec3<T> getFootVector(int foot) {
      return (_robot->getHipLocation(foot)
              + _data.legControllerData[foot]->p);
  }

  /**
   * Vector of the foot in the body frame
   */
  Vec3<T> getFootVelocity(int foot) {
      return _data.legControllerData[foot]->v;
  }

  /**
   * Position of the feet in the true world frame
   */
  Vec3<T> getStanceFootPosWorld(int foot) {
      return lastStanceFootPos.col(foot);
  }

  /**
   * Future footstep positions in the true world frame
   */
  Vec3<T> getFutureFootPosWorld(int foot) {
      return futureStanceFootPos.col(foot);
  }


  /**
   * Vector from the CoM to the foot in the body frame
   */
  Vec3<T> getAverageFootPosWorld() {
      Vec3<T> aveFootPos(0.0, 0.0, 0.0);
      for (int foot = 0; foot < _robot->NUM_FEET; foot++) {
          aveFootPos += getFootPosWorld(foot);
      }
      return aveFootPos / _robot->NUM_FEET;
  }


  /**
   * Vector from the CoM to the foot in the body frame
   */
  Vec3<T> getAverageStanceFootPosWorld() {
      Vec3<T> aveFootPos(0.0, 0.0, 0.0);
      for (int foot = 0; foot < _robot->NUM_FEET; foot++) {
          aveFootPos += lastStanceFootPos.col(foot);
      }
      return aveFootPos / _robot->NUM_FEET;
  }

  /*!
   * Set the contact phase
   */
  void setContactPhase(Vec4<T>& phase) { 
    *_data.contactPhase = phase; 
  }

  /*!
   * Add an estimator of the given type
   * @tparam EstimatorToAdd
   */
  template <typename EstimatorToAdd>
  void addEstimator() {
    auto* estimator = new EstimatorToAdd();
    estimator->setData(_data);
    estimator->setup();
    _estimators.push_back(estimator);
  }

  /*!
   * Remove all estimators of a given type
   * @tparam EstimatorToRemove
   */
  template <typename EstimatorToRemove>
  void removeEstimator() {
    int nRemoved = 0;
    _estimators.erase(
        std::remove_if(_estimators.begin(), _estimators.end(),
                       [&nRemoved](GenericEstimator<T>* e) {
                         if (dynamic_cast<EstimatorToRemove*>(e)) {
                           delete e;
                           nRemoved++;
                           return true;
                         } else {
                           return false;
                         }
                       }),
        _estimators.end());
  }

  /*!
   * Remove all estimators
   */
  void removeAllEstimators() {
    for (auto estimator : _estimators) {
      delete estimator;
    }
    _estimators.clear();
  }

  ~StateEstimatorContainer() {
    for (auto estimator : _estimators) {
      delete estimator;
    }
  }

 private:
  StateEstimatorData<T> _data;
  std::vector<GenericEstimator<T>*> _estimators;
  Vec4<T> _phase;
  Vec4<T> _footHeights;
  Mat34<T> lastStanceFootPos;
  Mat34<T> futureStanceFootPos;
  Quadruped<T>* _robot;
};

#endif  // PROJECT_STATEESTIMATOR_H
