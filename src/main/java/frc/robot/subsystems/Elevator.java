/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             *
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.commands.ElevatorControl;
import frc.robot.Robot;

public class Elevator extends Subsystem 
{
  //TALON SRX MOTORS
  private final WPI_TalonSRX m_elevatorOne, m_elevatorTwo, m_elevatorThree, m_elevatorFour;

  //SPEED CONTROLLER GROUP
  private final SpeedControllerGroup m_elevatorGroup;

  //DRIVETRAIN CONSTRUCTOR
  public Elevator() 
  {
    //SPARK MAX MOTORS
    this.m_elevatorOne = new WPI_TalonSRX(Robot.ROBOTMAP.m_elevatorOne);
    this.m_elevatorTwo = new WPI_TalonSRX(Robot.ROBOTMAP.m_elevatorTwo);
    this.m_elevatorThree = new WPI_TalonSRX(Robot.ROBOTMAP.m_elevatorThree);
    this.m_elevatorFour = new WPI_TalonSRX(Robot.ROBOTMAP.m_elevatorFour);

    //SPEED CONTROLLER GROUP
    this.m_elevatorGroup = new SpeedControllerGroup(this.m_elevatorOne, this.m_elevatorTwo, this.m_elevatorThree, this.m_elevatorFour);
  }

  @Override
  public void initDefaultCommand() 
  {
    setDefaultCommand(new ElevatorControl());
  }

  public void move(double speed)
  {
      this.m_elevatorGroup.set(speed);
  }

  //STOP ROBOT
  public void stop()
  {
    move(0);
  }
}