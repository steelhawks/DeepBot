/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DiffDrive extends Command 
{
  public DiffDrive() 
  {
    super("DiffDrive");
    requires(Robot.DRIVETRAIN);
  }

  @Override
  protected void initialize() {}

  @Override
  protected void execute() 
  {
    Robot.DRIVETRAIN.arcadeDrive(Robot.OI.js_drive);
  }

  @Override
  protected boolean isFinished() 
  {
    return false;
  }

  @Override
  protected void end() {}

  @Override
  protected void interrupted() 
  {
      Robot.DRIVETRAIN.stop();
  }
}