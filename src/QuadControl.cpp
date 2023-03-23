#include <cmath>
#include <iostream>

#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;

#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();

  // Load parameters (default to 0)
  kpPosXY = config->Get(_config + ".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);

  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to
  //   individual motor thrust commands
  // INPUTS:
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS:
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
  // cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
  // cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
  // cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right

  // Perpendicular distance to axes.
  const float l = L / sqrt(2);

  const float tau_x_bar = momentCmd.x / l;
  const float tau_y_bar = momentCmd.y / l;
  const float tau_z_bar = -momentCmd.z / kappa; // thrust up, momentCmd is down

  cmd.desiredThrustsN[0] = (collThrustCmd + tau_x_bar + tau_y_bar + tau_z_bar) / 4.0f; // front left
  cmd.desiredThrustsN[1] = (collThrustCmd - tau_x_bar + tau_y_bar - tau_z_bar) / 4.0f; // front right
  cmd.desiredThrustsN[2] = (collThrustCmd + tau_x_bar - tau_y_bar - tau_z_bar) / 4.0f; // rear left
  cmd.desiredThrustsN[3] = (collThrustCmd - tau_x_bar - tau_y_bar + tau_z_bar) / 4.0f; // rear right

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS:
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS:
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  const V3F error = pqrCmd - pqr;
  const V3F u_bar = kpPQR * error;
  momentCmd = V3F(Ixx, Iyy, Izz) * u_bar;
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS:
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS:
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // Reference: https://www.dynsyslab.org/wp-content/papercite-data/pdf/schoellig-acc12.pdf
  if (collThrustCmd > 0.f) {
    // Calculate desired acceleration in the inertial frame. Thrust is negated to account
    // for the Z axis pointing down in the body frame.
    const float c = -collThrustCmd / mass;
    // (R02, R12, R22) represent the direction of the collective thrust in the inertial frame.
    const V3F b(R(0,2),R(1,2),R(2,2));
    // Section 3 of the paper linked above shows to obtain the inertial frame accels
    // from the rotation matrix R, desired accel c, and g. By manipulating that equation,
    // we can calculate the desired matrix rotation matrix entries b_c.
    // Note that b_c.z will be ignored as we are not looking to control yaw here.
    V3F b_c = accelCmd / c;
    b_c.x = CONSTRAIN(b_c.x, -sin(maxTiltAngle), sin(maxTiltAngle));
    b_c.y = CONSTRAIN(b_c.y, -sin(maxTiltAngle), sin(maxTiltAngle));
    // Calculate matrix entries error and multiply by kpBank gain.
    const V3F b_dot_c = kpBank *  (b_c - b);
    // Calculate commanded angular velocities p_c and q_c using the identity
    // provided in the paper above.
    const float p_c = (V3F(R(1,0),-R(0,0), 0.f) * b_dot_c).sum() / R(2,2);
    const float q_c = (V3F(R(1,1),-R(0,1), 0.f) * b_dot_c).sum() / R(2,2);
    pqrCmd = V3F(p_c,q_c,0.f);
  } else {
    pqrCmd = V3F(0.f,0.f,0.f);
  }
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical
  //   acceleration feed-forward command
  // INPUTS:
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS:
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // Altitude controller is a PID controller.
  // gravitational acceleration.
  const float g = 9.81;
  // Accumulate Pos error.
  integratedAltitudeError += (posZCmd - posZ) * dt;
  // std::cout << "posZDifference: " << posZCmd - posZ << std::endl;
  // std::cout << "posZCmd: " << posZCmd << " posZ: " << posZ << std::endl;
  // Pos error gets fused into the commanded velocity.
  velZCmd += kpPosZ * (posZCmd - posZ);
  // Contrain commanded Z velocity. Not that both maxAscentRate and maxDescentRate are >=0.
  velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);
  // std::cout << "velZDiff: " << velZCmd - velZ << std::endl;
  // std::cout << " velZCmd: " << velZCmd << " velZ: " << velZ << std::endl;
  // Vel error and integrated altitude error contribute to commanded Z velocity.
  accelZCmd +=  kpVelZ *  (velZCmd - velZ) + KiPosZ * integratedAltitudeError;
  // Account for gravitational acceleration.
  accelZCmd -= g;
  // accelZCmd is in the vehicle's Z axis. Divide by R(2,2) to align thrust with the global frame.
  // thrust is directed upwards unlike the vehicle's NED frame, thus the negation of accelZCmd.
  // Multiply by mass since a force is expected (rather than an acceleration.)
  thrust = mass * -accelZCmd / R(2,2);
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS:
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations.
  //     the Z component should be 0
  // HINTS:
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // Lateral Position controller is a PD controller.
  // Pos error gets fused into the commanded velocity.
  velCmd += kpPosXY * (posCmd - pos);
  // Constrain commanded lateral velocity.
  velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
  velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);
  // Vel error gets fused into the commanded acceleration.
  accelCmd += kpVelXY * (velCmd - vel);
  // Contrain commanded lateral acceleration.
  accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
  accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS:
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS:
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b].
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd = 0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // Yaw controller is a P controller.
  // Calculate Yaw error and contrain yaw error from 0 to 2*PI.
  yawRateCmd = kpYaw * (fmodf(yawCmd - yaw, 2 * M_PI));
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;
}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f * (maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust + thrustMargin) * 4.f, (maxMotorThrust - thrustMargin) * 4.f);

  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);

  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
