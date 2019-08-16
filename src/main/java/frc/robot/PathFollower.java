/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import java.io.IOException;

public class PathFollower
{
  private EncoderFollower leftFollower;
  private EncoderFollower rightFollower;

  private Notifier follower;

  private boolean valid = false;

  /**Initiate pathweaver by loading the paths and creating all required objects. */
  public void init()
  {
    try
    { 
      Trajectory left_trajectory = PathfinderFRC.getTrajectory(Robot.ROBOTMAP.pathName + ".left");
      Trajectory right_trajectory = PathfinderFRC.getTrajectory(Robot.ROBOTMAP.pathName + ".right");

      this.leftFollower = new EncoderFollower(left_trajectory);
      this.rightFollower = new EncoderFollower(right_trajectory);

      this.leftFollower.configureEncoder(Robot.DRIVETRAIN.enc_leftO.get(), Robot.ROBOTMAP.ticksPerRev, Robot.ROBOTMAP.wheelDiameter);
      // You must tune the PID values on the following line!
      this.leftFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / Robot.ROBOTMAP.maxVelocity, 0);

      this.rightFollower.configureEncoder(Robot.DRIVETRAIN.enc_rightO.get(), Robot.ROBOTMAP.ticksPerRev, Robot.ROBOTMAP.wheelDiameter);
      // You must tune the PID values on the following line!
      this.rightFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / Robot.ROBOTMAP.maxVelocity, 0);

      this.follower = new Notifier(this::followPath);
      this.follower.startPeriodic(left_trajectory.get(0).dt);

      this.valid = true;
      System.out.println("Path was successfully loaded!");
    }
    catch (IOException e)
    {
      this.valid = false;
      System.out.println("Path failed to load...");
    }
  }

  /** Adjust the heading of the robot and motor group speeds in order to follow the path. */
  public void followPath() 
  {
    if (this.leftFollower.isFinished() || this.rightFollower.isFinished() || !this.valid) 
    {
      stop();
    } 
    else 
    {
      double left_speed = this.leftFollower.calculate(Robot.DRIVETRAIN.enc_rightO.get());
      double right_speed = this.rightFollower.calculate(Robot.DRIVETRAIN.enc_leftO.get());
      double heading = Robot.DRIVETRAIN.gyro.getAngle();
      double desired_heading = Pathfinder.r2d(this.leftFollower.getHeading());
      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      double turn =  0.8 * (-1.0/80.0) * heading_difference;
      Robot.DRIVETRAIN.m_leftGroup.set(left_speed + turn);
      Robot.DRIVETRAIN.m_rightGroup.set(right_speed - turn);
    }
  }

  /** Stop pathfollower and the robot. */
  public void stop()
  {
    this.follower.stop();
    Robot.DRIVETRAIN.stop();
  }

  /** Returns if the pathfollower has finished @return if patfollower has finished */
  public boolean isFinished()
  {
    return this.leftFollower.isFinished() && this.rightFollower.isFinished();
  }

  /** Checks if the loaded path has been found and is valid @return if the loaded path is found and valid */
  public boolean isValid()
  {
    return this.valid;
  }
}