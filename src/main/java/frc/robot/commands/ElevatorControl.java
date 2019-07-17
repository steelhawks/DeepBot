/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ElevatorControl extends Command 
{
  public ElevatorControl() 
  {
    requires(Robot.ELEVATOR);
  }

  @Override
  protected void initialize() {}

  @Override
  protected void execute() 
  {
    Robot.ELEVATOR.move(Robot.OI.js_operate.getRawAxis(Robot.ROBOTMAP.btn_translate));
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
      Robot.ELEVATOR.stop();
  }
}