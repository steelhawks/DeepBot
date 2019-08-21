/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class RobotMap 
{
  /*****
   * Identity variables
   *****/
  public final String robotName = "";

  /*****
   * Drivetrain variables
   *****/

  //Right Motor Ports
  public final int m_rightOne = 0;
  public final int m_rightTwo = 1;
  public final int m_rightThree = 2;

  //Left Motor Ports
  public final int m_leftOne = 3;
  public final int m_leftTwo = 4;
  public final int m_leftThree = 5;

  //Shifter Solenoid Ports
  public final int sol_shiftOn = 0;
  public final int sol_shiftOff = 1;

  //Optical Encoder Ports

  public final int enc_leftA = 0;
  public final int enc_leftB = 1; 
  public final int enc_rightA = 2;
  public final int enc_rightB = 3;

  //Gyro
  public final double gyro_constant = 0.008;

  //Max Acceleration Rate
  public final double maxAccelerationRate = 1.9;

  /*****
   * Elevator variables
   *****/

  //Motor Ports

  public final int m_elevatorOne = 6;
  public final int m_elevatorTwo = 7;
  public final int m_elevatorThree = 8;
  public final int m_elevatorFour = 9;

  //Limit Switch Ports
  public final int s_topLim = 0;
  public final int s_bottomLim = 1;

  /*****
   * Ultrasonic variables
   *****/

  public final int s_ultraPing = 8;
  public final int s_ultraEcho = 9;


  /*****
   * Driverstation variables
   *****/

  //Input Ports
  public final int js_one = 0;
  public final int js_two = 1;

  //Button ports driver
  public final int btn_shift = 1;
  public final int btn_align = 3;

  //Button ports gamepad
  public final int btn_translate = 1;

  /*****
   * Pathfollower variables
   *****/

  public final int ticksPerRev = 1024;
  public final double wheelDiameter = 6.0;
  public final double maxVelocity = 15.9;
  public final String pathName = "PathOne";
}
