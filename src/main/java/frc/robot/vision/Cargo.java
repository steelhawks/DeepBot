/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import frc.robot.Robot;
import frc.util.subsystems.VisionSubsystem;

public class Cargo extends VisionSubsystem
{
  private boolean initAlign;
  private boolean alignAngle;
  private double angle;
  private double xPosLeftLimit;
  private double xPosRightLimit;

  /** Aligns the robot. */
  public void align()
  {
    if (Robot.OI.js_drive.getRawButtonPressed(2))
    {
      disable();
    }
    else if (!this.initAlign)
    {
      if (Math.abs(Robot.DRIVETRAIN.gyro.getAngle()) < (getAngle() - 0.1) && getNTXPos("Cargo") < getXPosLeftLimit())
      {
        Robot.DRIVETRAIN.rotate(0.325);
      }
      else if(Math.abs(Robot.DRIVETRAIN.gyro.getAngle()) < (getAngle() - 0.1) && getNTXPos("Cargo") > getXPosRightLimit()) 
      {
        Robot.DRIVETRAIN.rotate(-0.325);
      }
      else 
      {
        Robot.DRIVETRAIN.resetGyro();
        Robot.DRIVETRAIN.stop();
        this.initAlign = true;
      }
    }
    else
    {
      if (!Robot.ULTRA.isClose())
      {
        Robot.DRIVETRAIN.gyroMoveStraight(Robot.DRIVETRAIN.decimalSpeed(Robot.ULTRA.getRangeInches()));
      }
      else
      {
        Robot.DRIVETRAIN.stop();
        alignAngle = true;
      }
    } 
  }

  /** Sets the target angle for alignment. */
  public void setAngle(double angle)
  {
    this.angle = angle;
  }

  /** Gets the target angle for alignment. @return The target angle for alignment */
  public double getAngle()
  {
    return this.angle;
  }

  /** Sets the x cooridnate of the left border of the center of the cargo. */
  public void setXPosLeftLimit(double xPosLeftLimit)
  {
    this.xPosLeftLimit = xPosLeftLimit;
  }

  /** Sets the x cooridnate of the right border of the center of the cargo. */
  public void setXPosRightLimit(double xPosRightLimit)
  {
    this.xPosRightLimit = xPosRightLimit;
  }

  /** Gets the x cooridnate of the left border of the center of the cargo. @return The x cooridnate of the left border of the cargo */
  public double getXPosLeftLimit()
  {
    return this.xPosLeftLimit;
  }

  /** Gets the x cooridnate of the right border of the center of the cargo @return The x cooridnate of the right border of the cargo */
  public double getXPosRightLimit()
  {
    return this.xPosRightLimit;
  }

  /** Returns the difference between the right or left border and center of the cargo. The difference is the smallest between the two. 
   *  @return The difference between the right or left border and center of the cargo */
  public double getXPosDiff(double xPos)
  {
    if (xPos > 160)
    {
      return xPos - 160;
    }
    return Math.abs(160 - xPos);
  }

  /** Returns true if alignment and distancing to the center of the cargo is complete. @return If alignment and distancing to the center of the cargo is complete */
  public boolean isAligned()
  {
    return this.alignAngle;
  }

  @Override
  public void enable()
  {
    alignAngle = false;
    initAlign = false;
  }

  @Override
  public void disable()
  {
    Robot.DRIVETRAIN.stop();
    this.alignAngle = true;
  }

  @Override
  public void ping()
  {
    getNTXPos("Cargo");
    getNTYPos("Cargo");
    getNTAngle("Cargo");
    getNTDistance("Cargo");
  }

  @Override
  public boolean isAlive()
  {
    return true;
  }
}