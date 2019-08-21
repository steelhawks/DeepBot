/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ShiftGear extends Command 
{

  public ShiftGear() 
  {
    super("ShiftGear");
    requires(Robot.DRIVETRAIN);
  }

  @Override
  protected void initialize() {}

  @Override
  protected void execute() 
  {
    Robot.DRIVETRAIN.shiftGear();
  }
    
  @Override
  protected boolean isFinished() 
  {
    return true;
  }

  @Override
  protected void end() {}

  @Override
  protected void interrupted() {}
}