/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot;
import frc.robot.commands.DiffDrive;
import frc.util.subsystems.MechanicalSubsystem;

public class Drivetrain extends MechanicalSubsystem 
{
  //SPARK MAX MOTORS
  private final CANSparkMax m_leftOne, m_leftTwo, m_leftThree, m_rightOne, m_rightTwo, m_rightThree;

  //SPEED CONTROLLER GROUPS
  public final SpeedControllerGroup m_leftGroup, m_rightGroup;

  //DIFFERENTIAL DRIVE
  private final DifferentialDrive diffDrive;

  //SHIFTING SOLENOIDS
  private final DoubleSolenoid sol_shift;

  //NAVX MXP gyro
  public final AHRS gyro;

  //NEO MOTOR ENCODERS
  public CANEncoder enc_left, enc_right;

  //GRAYHILL OPTICAL ENCODERS
  public Encoder enc_leftO, enc_rightO;

  /** Drivetrain constructor. */
  public Drivetrain() 
  {
    //SPARK MAX LEFT MOTORS
    this.m_leftOne = new CANSparkMax(Robot.ROBOTMAP.m_leftOne, MotorType.kBrushless);
    this.m_leftTwo = new CANSparkMax(Robot.ROBOTMAP.m_leftTwo, MotorType.kBrushless);
    this.m_leftThree = new CANSparkMax(Robot.ROBOTMAP.m_leftThree, MotorType.kBrushless);
    
    //SPARK MAX RIGHT MOTORS
    this.m_rightOne = new CANSparkMax(Robot.ROBOTMAP.m_rightOne, MotorType.kBrushless);
    this.m_rightTwo = new CANSparkMax(Robot.ROBOTMAP.m_rightTwo, MotorType.kBrushless);
    this.m_rightThree = new CANSparkMax(Robot.ROBOTMAP.m_rightThree, MotorType.kBrushless);

    //SPEED CONTROLLER GROUPS
    this.m_leftGroup = new SpeedControllerGroup(this.m_leftOne, this.m_leftTwo, this.m_leftThree);
    this.m_rightGroup = new SpeedControllerGroup(this.m_rightOne, this.m_rightTwo, this.m_rightThree);

    //DIFFERENTIAL DRIVE
    this.diffDrive = new DifferentialDrive(this.m_leftGroup, this.m_rightGroup);

    //NAVX MXP GYRO
    this.gyro = new AHRS(SPI.Port.kMXP);

    //SHIFTING SOLENOIDS
    this.sol_shift = new DoubleSolenoid(Robot.ROBOTMAP.sol_shiftOn, Robot.ROBOTMAP.sol_shiftOff);

    //NEO MOTOR ENCODERS
    this.enc_left = this.m_leftOne.getEncoder();
    this.enc_right = this.m_rightOne.getEncoder();

    //GRAYHILL OPTICAL ENCODERS
    this.enc_leftO = new Encoder(Robot.ROBOTMAP.enc_leftA, Robot.ROBOTMAP.enc_leftB, false, EncodingType.k4X);
    this.enc_rightO = new Encoder(Robot.ROBOTMAP.enc_rightA, Robot.ROBOTMAP.enc_rightB, false, EncodingType.k4X);

    //SET ROBOT TO LOW GEAR
    sol_shift.set(DoubleSolenoid.Value.kReverse);

    //SET MOTORS TO BREAK
    setMotorsBrake();

    //SET MAX LOOP RATE
    setAccelerationRate(Robot.ROBOTMAP.maxAccelerationRate);
  }

  @Override
  public void initDefaultCommand() 
  {
    setDefaultCommand(new DiffDrive());
  }

  /** Drives the robot with the arcadeDrive() method for differental drive robots. */
  public void arcadeDrive(Joystick stick) 
  {
    this.diffDrive.arcadeDrive(stick.getY(), -stick.getTwist());
  }

  /** Toggles between high and low gear. */
  public void shiftGear() 
  {
    if(sol_shift.get() == DoubleSolenoid.Value.kForward) 
    {
      this.sol_shift.set(DoubleSolenoid.Value.kReverse);
    } 
    else 
    {
      this.sol_shift.set(DoubleSolenoid.Value.kForward);
    }
  }

  /** Rotates the robot. */
  public void rotate(double speed)
  {
    this.m_leftGroup.set(speed);
    this.m_rightGroup.set(speed);
  }

  public boolean stop()
  {
    rotate(0);
    return true;
  }

  public void ping() {}

  public boolean isAlive()
  {
    return this.diffDrive.isAlive();
  }

  /** Set motors to coast. */
  public void setMotorsCoast()
  {
    this.m_leftOne.setIdleMode(IdleMode.kCoast);
    this.m_leftTwo.setIdleMode(IdleMode.kCoast);
    this.m_leftThree.setIdleMode(IdleMode.kCoast);
    this.m_rightOne.setIdleMode(IdleMode.kCoast);
    this.m_rightTwo.setIdleMode(IdleMode.kCoast);
    this.m_rightThree.setIdleMode(IdleMode.kCoast);
  }

  /** Sets motors to brake. */
  public void setMotorsBrake()
  {
    this.m_leftOne.setIdleMode(IdleMode.kCoast);
    this.m_leftTwo.setIdleMode(IdleMode.kCoast);
    this.m_leftThree.setIdleMode(IdleMode.kBrake);
    this.m_rightOne.setIdleMode(IdleMode.kCoast);
    this.m_rightTwo.setIdleMode(IdleMode.kCoast);
    this.m_rightThree.setIdleMode(IdleMode.kBrake);
  }

  /** Sets max acceleration. */
  public void setAccelerationRate(double rate)
  {
    this.m_leftOne.setOpenLoopRampRate(rate);
    this.m_leftTwo.setOpenLoopRampRate(rate);
    this.m_leftThree.setOpenLoopRampRate(rate);
    this.m_rightOne.setOpenLoopRampRate(rate);
    this.m_rightTwo.setOpenLoopRampRate(rate);
    this.m_rightThree.setOpenLoopRampRate(rate);
  }

  /** Gets the left encoder position @return left encoder position */
  public double getLeftEncPosition() 
  {
    return this.enc_left.getPosition();
  }

  /** Gets the left encoder velcoity @return left encoder velocity */
  public double getLeftEncVelocity() 
  {
    return this.enc_left.getVelocity();
  }

  /** Gets the right encoder position @return right encoder position */
  public double getRightEncPosition() 
  {
    return this.enc_right.getPosition();
  }

  /** Gets the right encoder velocity @return right encoder velocity */
  public double getRightEncVelocity() 
  {
    return this.enc_right.getVelocity();
  }

  /**
   * Reset the robot gyro.
   */
  public void resetGyro() 
  {
    this.gyro.reset();
    this.gyro.zeroYaw();
  }
}