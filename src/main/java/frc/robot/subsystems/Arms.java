/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Robot;
import frc.util.subsystems.MechanicalSubsystem;

public class Arms extends MechanicalSubsystem 
{

  //ARM MOTOR TALONS
  public WPI_TalonSRX m_arm;

  //ARMS CONSTRUCTOR
  public Arms() 
  {
    this.m_arm = new WPI_TalonSRX(Robot.ROBOTMAP.m_arm);
  }

  //DEFAULT COMMAND
  @Override
  public void initDefaultCommand() {}

  //INTAKING METHOD
  public void armIntakeButton() 
  {
    if(m_arm.get() == 0.0) 
    {
      m_arm.set(Robot.ROBOTMAP.armSpeed);
    } 
    else 
    {
      m_arm.set(0.0);
    }
  }

  //SHOOTING METHOD
  public void armOuttakeButton() 
  {
    if(m_arm.get() == 0.0) 
    {
      m_arm.set(-Robot.ROBOTMAP.armSpeed);
    } 
    else 
    {
      m_arm.set(0.0);
    }
  }

  //STOPPING METHOD
  public boolean stop()
  {
    m_arm.set(0.0);
    return true;
  }

  public void ping() {}

  public boolean isAlive()
  {
    return true;
  }
}
