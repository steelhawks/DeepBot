/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Align;
import frc.robot.sensors.Ultra;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.vision.Tape;

public class Robot extends TimedRobot
{
  /*****
   * Robot Objects
   *****/
  public static RobotMap ROBOTMAP;
  public static OI OI;
  public static PathFollower PATHFOLLOWER;
  public static Drivetrain DRIVETRAIN;
  public static Elevator ELEVATOR;
  public static Tape TAPE;
  public static Ultra ULTRA;

  /*****
   * Auto Variables
   *****/

  private boolean autoFinished;

  @Override
  public void robotInit() 
  {
    ROBOTMAP = new RobotMap();
    OI = new OI();
    PATHFOLLOWER = new PathFollower();
    DRIVETRAIN = new Drivetrain();
    ELEVATOR = new Elevator();
    TAPE = new Tape();
    ULTRA = new Ultra();
    Robot.PATHFOLLOWER.init();
    this.autoFinished = false;
  }

  @Override
  public void robotPeriodic() 
  {
    SmartDashboard.putString("Match Time", returnTime());
    SmartDashboard.putBoolean("Valid Path", Robot.PATHFOLLOWER.isValid());
    SmartDashboard.putNumber("Left Motor Group", Robot.DRIVETRAIN.m_leftGroup.get());
    SmartDashboard.putNumber("Right Motor Group", Robot.DRIVETRAIN.m_rightGroup.get());
    SmartDashboard.putNumber("Gyro Angle", Robot.DRIVETRAIN.gyro.getAngle());
  }

  @Override
  public void disabledInit() 
  {
    Scheduler.getInstance().removeAll();
    Scheduler.getInstance().disable();
    DRIVETRAIN.stop();
    Robot.PATHFOLLOWER.stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() 
  {
    if (!this.autoFinished)
    {
      while(!Robot.PATHFOLLOWER.isFinished())
      {
        Robot.PATHFOLLOWER.followPath();
      }
      new Align();
      this.autoFinished = true;
    }
  }

  @Override
  public void teleopInit() 
  {
    Robot.PATHFOLLOWER.stop();
    Scheduler.getInstance().enable();
  }

  @Override
  public void teleopPeriodic() 
  {
    Scheduler.getInstance().run();
  }

  @Override
  public void testPeriodic() {}
  
  public String returnTime() 
  {
    boolean isAuton = DriverStation.getInstance().isAutonomous();
    int time = (int) DriverStation.getInstance().getMatchTime();

    if (time == -1) {
      time = 0;
    }

    String minutes = Integer.toString(time / 60);
    String seconds = Integer.toString(time % 60);

    if (time % 60 < 10) 
    {
      seconds = "0" + seconds;
    }

    if (isAuton) 
    {
      return "Sandstorm: " + minutes + ":" + seconds;
    }
    else 
    {
      return "Teleop: " + minutes + ":" + seconds;
    }
  }
}
