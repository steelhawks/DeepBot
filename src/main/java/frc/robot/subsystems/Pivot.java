/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.Robot;
import frc.robot.commands.PivotControl;
import frc.util.subsystems.MechanicalSubsystem;

public class Pivot extends MechanicalSubsystem
{
  //TALON SRX MOTORS
  private final WPI_TalonSRX m_pivot;

  //SPEED CONTROLLER GROUP
  private final SpeedControllerGroup m_pivotGroup;

  /** Elevator constructor */
  public Pivot() 
  {
    //SPARK MAX MOTORS
    this.m_pivot = new WPI_TalonSRX(Robot.ROBOTMAP.m_pivot);

    //SPEED CONTROLLER GROUP
    this.m_pivotGroup = new SpeedControllerGroup(this.m_pivot);
  }

  @Override
  public void initDefaultCommand() 
  {
    setDefaultCommand(new PivotControl());
  }

  /** Moves the elevator up or down. */
  public void move(double speed)
  {
    this.m_pivotGroup.set(0);
  }

  public boolean stop()
  {
    move(0);
    return true;
  }

  public void ping() {}

  public boolean isAlive()
  {
    return true;
  }
}