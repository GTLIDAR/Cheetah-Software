/*! @file A1.h
 *  @brief Utility function to build an A1 object
 *
 * This file is based on a1.urdf provided by Unitree and builds a model
 * of the A1 robot.  The inertia parameters of all bodies are
 * determined from its urdf file.
 *
 */

#ifndef DYNAMICS_A1_H
#define DYNAMICS_A1_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of A1
 */
template <typename T>
Quadruped<T> buildA1() {
    Quadruped<T> a1;
    a1._robotType = RobotType::A1;

    a1._bodyMass = 4.713;
    a1._bodyLength = 0.1805 * 2;
    a1._bodyWidth = 0.047 * 2;
    a1._bodyHeight = 0.05 * 2;
    a1._abadGearRatio = 6; // not sure
    a1._hipGearRatio = 6; // not sure
    a1._kneeGearRatio = 9.33; // not sure
    a1._abadLinkLength = 0.0838;
    a1._hipLinkLength = 0.2;
    a1._kneeLinkY_offset = 0.0;
    a1._kneeLinkLength = 0.2;
    a1._maxLegLength = 0.4;


    a1._motorTauMax = 33.5f;
    a1._batteryV = 21.6;
    a1._motorKT = .05;  // this is flux linkage * pole pairs // not sure
    a1._motorR = 0.173; // not sure
    a1._jointDamping = .0;
    a1._jointDryFriction = .0;


    // rotor inertia if the rotor is oriented so it spins around the z-axis (not sure)
    Mat3<T> rotorRotationalInertiaZ;
    rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
    rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

    Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
    Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
    Mat3<T> rotorRotationalInertiaX =
            RY * rotorRotationalInertiaZ * RY.transpose();
    Mat3<T> rotorRotationalInertiaY =
            RX * rotorRotationalInertiaZ * RX.transpose();

    // spatial inertias
    Mat3<T> abadRotationalInertia;
    abadRotationalInertia << 469.246, 9.409, -0.342, 9.409, 807.49, 0.466, -0.342, 0.466, 552.929;
    abadRotationalInertia = abadRotationalInertia * 1e-6;
    Vec3<T> abadCOM(-0.003311, -0.000635, 3.1e-05);  // LEFT
    SpatialInertia<T> abadInertia(0.696, abadCOM, abadRotationalInertia);

    Mat3<T> hipRotationalInertia;
    hipRotationalInertia << 5529.065, -4.825, 343.869, -4.825, 5139.339, -22.448, 343.869, -22.448, 1367.788;
    hipRotationalInertia = hipRotationalInertia * 1e-6;
    Vec3<T> hipCOM(-0.003237, 0.022327, -0.027326);
    SpatialInertia<T> hipInertia(1.013, hipCOM, hipRotationalInertia);

    Mat3<T> kneeRotationalInertia;
    kneeRotationalInertia << 2997.972, 0, -141.163, 0, 3014.022, 0, -141.163, 0, 32.426;
    kneeRotationalInertia = kneeRotationalInertia * 1e-6;
    Vec3<T> kneeCOM(0.006435, 0.0, -0.107388);
    SpatialInertia<T> kneeInertia(0.166, kneeCOM, kneeRotationalInertia);

    Vec3<T> rotorCOM(0, 0, 0); // not sure
    SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
    SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

    Mat3<T> bodyRotationalInertia;
    bodyRotationalInertia << 16839.93, 83.902, 597.679, 83.902, 56579.028, 25.134, 597.679, 25.134, 64713.601;
    bodyRotationalInertia = bodyRotationalInertia * 1e-6;
    Vec3<T> bodyCOM(0.012731, 0.002186, 0.000515);
    SpatialInertia<T> bodyInertia(a1._bodyMass, bodyCOM,
                                  bodyRotationalInertia);

    a1._abadInertia = abadInertia;
    a1._hipInertia = hipInertia;
    a1._kneeInertia = kneeInertia;
    a1._abadRotorInertia = rotorInertiaX;
    a1._hipRotorInertia = rotorInertiaY;
    a1._kneeRotorInertia = rotorInertiaY;
    a1._bodyInertia = bodyInertia;

    // locations
    a1._abadRotorLocation = Vec3<T>(a1._bodyLength, a1._bodyWidth, 0) * 0.5; // not sure
    a1._abadLocation =
            Vec3<T>(0.183, 0.047, 0);
    a1._hipLocation = Vec3<T>(0, 0.08505, 0);
    a1._hipRotorLocation = Vec3<T>(0, a1._abadLinkLength, 0);
    a1._kneeLocation = Vec3<T>(0, 0, -a1._hipLinkLength);
    a1._kneeRotorLocation = Vec3<T>(0, 0, 0);

    return a1;
}

#endif  // DYNAMICS_A1_H
