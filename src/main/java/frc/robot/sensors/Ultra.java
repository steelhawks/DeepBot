/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.Robot;
import frc.util.subsystems.SensorSubsystem;

public class Ultra extends SensorSubsystem
{
  private final Ultrasonic ULTRA = new Ultrasonic(Robot.ROBOTMAP.s_ultraPing, Robot.ROBOTMAP.s_ultraEcho);

  public boolean isCloseShift()
  {
    if(this.ULTRA.getRangeInches() <= 70)
    {
      return true;
    }
    return false;
  }

  public boolean isClose()
  {
    if(this.ULTRA.getRangeInches() <= 40)
    {
      return true;
    }
    return false;
  }

  public double getRangeInches()
  {
    return this.ULTRA.getRangeInches();
  }

  public boolean isRangeValid()
  {
    return this.ULTRA.isRangeValid();
  }

  @Override
  public void enable()
  {
    this.ULTRA.setEnabled(true);
    this.ULTRA.setAutomaticMode(true);
  }

  @Override
  public void disable()
  {
    this.ULTRA.setEnabled(false);
  }

  @Override
  public void ping()
  {
    this.ULTRA.ping();
  }

  @Override
  public boolean isAlive()
  {
    return isRangeValid();
  };
}