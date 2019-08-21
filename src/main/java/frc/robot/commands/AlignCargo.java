/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AlignCargo extends Command
{
  public AlignCargo() 
  {
    super("AlignCargo");
    requires(Robot.DRIVETRAIN);
    requires(Robot.TAPE);
    requires(Robot.ULTRA);
  }

  @Override
  protected void initialize()
  {
    Robot.DRIVETRAIN.resetGyro();
    Robot.DRIVETRAIN.setAcceleration(Robot.ROBOTMAP.maxAccelerationRate);
    Robot.TAPE.enable();
    Robot.ULTRA.enable();
    Robot.TAPE.setAngle(Robot.TAPE.getNTAngle("Cargo"));
    Robot.TAPE.setXPosLeftLimit(157.5);
    Robot.TAPE.setXPosRightLimit(162.5);
  }

  @Override
  protected void execute()
  {
    Robot.TAPE.align();
  }

  @Override
  protected boolean isFinished()
  {
    return Robot.TAPE.isAligned();
  }

  @Override
  protected void end()
  {
    Robot.DRIVETRAIN.stop();
    Robot.DRIVETRAIN.setAcceleration(0.0);
    Robot.TAPE.disable();
    Robot.ULTRA.disable();
  }

  @Override
  protected void interrupted()
  {
    end();
  }
}