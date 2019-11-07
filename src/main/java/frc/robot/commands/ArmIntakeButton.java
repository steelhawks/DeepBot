/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ArmIntakeButton extends Command 
{

  public ArmIntakeButton() 
  {
    super("ArmIntake");
    requires(Robot.ARMS);
  }

  @Override
  protected void initialize() {}

  @Override
  protected void execute() 
  {
    Robot.ARMS.armIntakeButton();
  }

  @Override
  protected boolean isFinished() 
  {
    return true;
  }

 @Override
  protected void end() 
  {
    Robot.ARMS.stop();
  }

  @Override
  protected void interrupted() 
  {
    end();
  }
}
