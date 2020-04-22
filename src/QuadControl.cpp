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
  kpPosXY = config->Get(_config+".kpPosXY", 0);
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
  
    float l = L * M_SQRT1_2; // l = L / sqrt(2)  

    /*
    //  So, Ixx, Iyy, Izz are the moments of inertia. "Kappa" is instead the ratio between thrust [N] and torque due to drag [N m], which is essentially k_f/k_m

    float c_bar = -collThrustCmd * mass / k_f;
    float p_bar = momentCmd.x * Ixx / (k_f * l);
    float q_bar = momentCmd.y * Iyy / (k_f * l);
    float r_bar = momentCmd.z * Izz / k_m;

    // Fn = kf * omegan
    float omega_4 = (c_bar + p_bar - r_bar - q_bar) / 4.f;
    float omega_3 = (r_bar - p_bar) / 2.f + omega_4;
    float omega_2 = (c_bar - p_bar) / 2.f - omega_3;
    float omega_1 = c_bar - omega_2 - omega_3 - omega_4;

  cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
  cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
  cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
  cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right

    cmd.desiredThrustsN[0] = -sqrt(omega_1); // front left
    cmd.desiredThrustsN[1] = sqrt(omega_2); // front right
    cmd.desiredThrustsN[2] = sqrt(omega_3); // rear left
    cmd.desiredThrustsN[3] = sqrt(omega_4); // rear right

    float F_tot = F0 + F1 + F2 + F3;
    tau_x = (F0 - F1 + F2 - F3) * l;                 // This is Roll
    tau_y = (F0 + F1 - F2 - F3) * l;                 // This is Pitch
    tau_z = (-F0 + F1 + F2 - F3) * kappa;
    // This is Yaw where l = L / sqrt(2)   -  since, unlike in lecture, L is defined as half the distance between rotors

    // Now we have 4 equations and 4 unknowns (the F values), so we can solve them to get the F1, F2, F3 and F4 values, given that we already know the F_tot, tau_x, tau_y and tau_z inputs to the GenerateMotorCommands(float collThrustCmd, V3F momentCmd) function
    */

    float c_bar = collThrustCmd;
    float p_bar = momentCmd.x / l;
    float q_bar = momentCmd.y / l;
    float r_bar = -momentCmd.z / kappa;

    cmd.desiredThrustsN[0] = (c_bar + p_bar + q_bar + r_bar) / 4.f; // front left
    cmd.desiredThrustsN[1] = (c_bar - p_bar + q_bar - r_bar) / 4.f; // front right
    cmd.desiredThrustsN[2] = (c_bar + p_bar - q_bar - r_bar) / 4.f; // rear left
    cmd.desiredThrustsN[3] = (c_bar - p_bar - q_bar + r_bar) / 4.f; // rear right

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

  // P controller
  /*
  float p_error = pqrCmd.x - pqr.x;
  float u_bar_p = kpPQR.x * p_error;

  float q_error = pqrCmd.y - pqr.y;
  float u_bar_q = kpPQR.y * q_error;

  float r_error = pqrCmd.z - pqr.z;
  float u_bar_r = kpPQR.z * r_error;

  momentCmd.x = Ixx * u_bar_p;
  momentCmd.y = Iyy * u_bar_q;
  momentCmd.z = Izz * u_bar_r;
  */
  V3F pqrError = pqrCmd - pqr;

  V3F I = V3F(Ixx, Iyy, Izz);
  
  V3F pqrDot = kpPQR * pqrError;

  momentCmd = I * pqrDot;

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

  if (collThrustCmd > 0.f) {
      // you need to first calculate the b_x_p term and b_y_p term. Both these terms are simply kp times the error,
      // if we just consider the x term, the actual value is R13 and the target value would be calculated
      // taking in account the collThrustCmd and accelCmd parameters

        float acc = -collThrustCmd / mass;
        float b_x_c_target = CONSTRAIN(accelCmd.x / acc, -maxTiltAngle, maxTiltAngle);
        float b_y_c_target = CONSTRAIN(accelCmd.y / acc, -maxTiltAngle, maxTiltAngle);

        float b_x_err = b_x_c_target - R(0, 2);
        float b_x_p_term = kpBank * b_x_err;

        float b_y_err = b_y_c_target - R(1, 2);
        float b_y_p_term = kpBank * b_y_err;

        float b_x_commanded_dot = b_x_p_term;
        float b_y_commanded_dot = b_y_p_term;

        pqrCmd.x = (R(1, 0) * b_x_commanded_dot - R(0, 0) * b_y_commanded_dot) / R(2, 2);
        pqrCmd.y = (R(1, 1) * b_x_commanded_dot - R(0, 1) * b_y_commanded_dot) / R(2, 2);

  } else {

      pqrCmd.x = 0.0;
      pqrCmd.y = 0.0;

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

  float posZerror = posZCmd - posZ;
  float velZError = velZCmd - velZ;
  integratedAltitudeError += posZerror * dt;

  // PID controller
  float acceleration_cmd = kpPosZ * posZerror + KiPosZ * integratedAltitudeError + kpVelZ * velZError + accelZCmd;

  float acceleration = (acceleration_cmd - CONST_GRAVITY) / R(2, 2);

  thrust = - mass * CONSTRAIN(acceleration, -maxAscentRate / dt, maxDescentRate / dt);

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

  V3F posError = posCmd - pos;
  V3F velError = velCmd - vel;

    // Limit speed
    if (velCmd.mag() > maxSpeedXY) {
        velCmd = velCmd.norm() * maxSpeedXY;
    }

    // PD controller
    accelCmd = kpPosXY * posError + kpVelXY * velError + accelCmd;

    if (accelCmd.mag() > maxAccelXY) {
        accelCmd = accelCmd.norm() * maxAccelXY;
    }

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

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Ensure the target is within range of 0 to 2 * pi
    if (yawCmd > 0) {
        yawCmd = fmodf(yawCmd, M_2_PI);
    } else {
        yawCmd = fmodf(yawCmd, -M_2_PI);
    }

    float yawError = yawCmd - yaw;
    if (yawError > M_PI) {
        yawError = yawError - M_2_PI;
    } else if (yawError < -M_PI) {
        yawError = yawError + M_2_PI;
    }

    // P controller
    yawRateCmd = kpYaw * yawError;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
