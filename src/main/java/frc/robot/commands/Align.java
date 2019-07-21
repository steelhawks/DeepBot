/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Align extends Command
{
    public Align() {
        requires(Robot.DRIVETRAIN);
        requires(Robot.VISION);
        requires(Robot.SENSORS);
    }

    @Override
    protected void initialize()
    {
        Robot.DRIVETRAIN.resetGyro();
        Robot.VISION.reset();
        Robot.VISION.setAngle(Robot.VISION.getNTAngle());
        Robot.VISION.setXPosLeftLimit(157.5);
        Robot.VISION.setXPosRightLimit(162.5);
        Robot.SENSORS.enable();
    }

    @Override
    protected void execute()
    {
        Robot.VISION.align();
    }

    @Override
    protected boolean isFinished()
    {
        return Robot.VISION.isAligned();
    }

    @Override
    protected void end()
    {
        Robot.DRIVETRAIN.stop();
        Robot.SENSORS.disable();
    }

    @Override
    protected void interrupted()
    {
        Robot.DRIVETRAIN.stop();
        Robot.SENSORS.disable();
    }
}