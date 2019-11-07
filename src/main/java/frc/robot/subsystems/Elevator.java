/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.commands.ElevatorControl;
import frc.robot.Robot;
import frc.util.subsystems.MechanicalSubsystem;

public class Elevator extends MechanicalSubsystem
{
  //TALON SRX MOTORS
  private final WPI_TalonSRX m_elevator;

  //SPEED CONTROLLER GROUP
  private final SpeedControllerGroup m_elevatorGroup;

  //LIIMIT SWITCHES
  private final DigitalInput s_topLim, s_bottomLim;

  /** Elevator constructor */
  public Elevator() 
  {
    //SPARK MAX MOTORS
    this.m_elevator = new WPI_TalonSRX(Robot.ROBOTMAP.m_elevator);

    //SPEED CONTROLLER GROUP
    this.m_elevatorGroup = new SpeedControllerGroup(this.m_elevator);

    //LIMIT SWITCHES
    this.s_topLim = new DigitalInput(Robot.ROBOTMAP.s_topLim);
    this.s_bottomLim = new DigitalInput(Robot.ROBOTMAP.s_bottomLim);
  }

  @Override
  public void initDefaultCommand() 
  {
    setDefaultCommand(new ElevatorControl());
  }

  /** Moves the elevator up or down. */
  public void move(double speed)
  {
    if (!this.s_topLim.get() && speed > 0)
    {
      this.m_elevatorGroup.set(speed);
    }
    else if (!this.s_bottomLim.get() && speed < 0)
    {
      this.m_elevatorGroup.set(speed);
    }
    else
    {
        this.m_elevatorGroup.set(0);
    }
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