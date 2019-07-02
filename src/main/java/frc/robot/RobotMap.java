/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class RobotMap 
{
  /*****
   * Drivetrain instance variables
   *****/

  //Right Motor Ports
  public final int RIGHT_MOTOR_PORT_ONE = 0;
  public final int RIGHT_MOTOR_PORT_TWO = 1;
  public final int RIGHT_MOTOR_PORT_THREE = 2;

  //Left Motor Ports
  public final int LEFT_MOTOR_PORT_ONE = 3;
  public final int LEFT_MOTOR_PORT_TWO = 4;
  public final int LEFT_MOTOR_PORT_THREE = 5;

  //Shifter Solenoid Ports
  public final int SHIFT_PORT_ON = 0;
  public final int SHIFT_PORT_OFF = 1;

  //Optical Encoder Ports

  public final int LEFT_ENC_PORT_A = 0;
  public final int LEFT_ENC_PORT_B = 1; 
  public final int RIGHT_ENC_PORT_A = 2;
  public final int RIGHT_ENC_PORT_B = 3;

  //Gyro
  public final double KP_GYRO = 0.008;

  /*****
   * Driverstation instance variables
   *****/

  //Input Ports
  public final int JOYSTICK_PORT_ONE = 0;

  //Button ports
  public final int SHIFT_BUTTON = 1;

  /*****
   * Constructor methods
   *****/
  public RobotMap() {}
}
