/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             *
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
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.commands.DiffDrive;
import frc.robot.Robot;

public class Drivetrain extends Subsystem 
{
  //SPARK MAX MOTORS
  private final CANSparkMax m_leftOne, m_leftTwo, m_leftThree, m_rightOne, m_rightTwo, m_rightThree;

  //SPEED CONTROLLER GROUPS
  private final SpeedControllerGroup m_leftGroup, m_rightGroup;

  //DIFFERENTIAL DRIVE
  private final DifferentialDrive diffDrive;

  //SHIFTING SOLENOIDS
  private final DoubleSolenoid sol_shift;

  //NAVX MXP gyro
  private final AHRS gyro;
  private final double gyro_constant;

  //NEO MOTOR ENCODERS
  public CANEncoder enc_left, enc_right;

  //DRIVETRAIN CONSTRUCTOR
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
    this.gyro_constant = Robot.ROBOTMAP.gyro_constant;

    //SHIFTING SOLENOIDS
    this.sol_shift = new DoubleSolenoid(Robot.ROBOTMAP.sol_shiftOn, Robot.ROBOTMAP.sol_shiftOff);

    //NEO MOTOR ENCODERS
    this.enc_left = this.m_leftOne.getEncoder();
    this.enc_right = this.m_rightOne.getEncoder();

    //SET ROBOT TO LOW GEAR
    sol_shift.set(DoubleSolenoid.Value.kReverse);

    //SET MOTORS TO BREAK
    setMotorsBrake();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DiffDrive());
  }

  //DRIVING METHOD
  public void arcadeDrive(Joystick stick) {
    this.diffDrive.arcadeDrive(stick.getY(), -stick.getTwist());
  }

  //SHIFTING METHOD -- if { LOW } else { HIGH }
  public void shiftGear() {
    if(sol_shift.get() == DoubleSolenoid.Value.kForward) {
      this.sol_shift.set(DoubleSolenoid.Value.kReverse);
    } else {
      this.sol_shift.set(DoubleSolenoid.Value.kForward);
    }
  }

  //MOVING STRAIGHT USING THE gyro METHOD
  public void gyroMoveStraight(double speed)
  {
    this.diffDrive.arcadeDrive(speed, -this.gyro.getAngle() * this.gyro_constant);
  }

  //MOVING STRAIGHT USING gyro AND ANGLE VALUE METHOD
  public void gyroMoveStraight(double speed, double angle)
  {
    this.diffDrive.arcadeDrive(-speed, -angle * this.gyro_constant);
  }

  //ROTATE ROBOT
  public void rotate(double speed)
  {
    this.m_leftGroup.set(speed);
    this.m_rightGroup.set(speed);
  }

  //STOP ROBOT
  public void stop()
  {
    rotate(0);
  }

  //CONVERT AN INT SPEED INTO A DECIMAL SPEED
  public double decimalSpeed(double speed)
  {
    return ((int)(((speed + 350) / 700.0) * 100) / 100.0);
  }

  //SET MOTORS TO COAST
  public void setMotorsCoast()
  {
    this.m_leftOne.setIdleMode(IdleMode.kCoast);
    this.m_leftTwo.setIdleMode(IdleMode.kCoast);
    this.m_leftThree.setIdleMode(IdleMode.kCoast);
    this.m_rightOne.setIdleMode(IdleMode.kCoast);
    this.m_rightTwo.setIdleMode(IdleMode.kCoast);
    this.m_rightThree.setIdleMode(IdleMode.kCoast);
  }

  //SET MOTORS TO BRAKE
  public void setMotorsBrake()
  {
    this.m_leftOne.setIdleMode(IdleMode.kCoast);
    this.m_leftTwo.setIdleMode(IdleMode.kCoast);
    this.m_leftThree.setIdleMode(IdleMode.kBrake);
    this.m_rightOne.setIdleMode(IdleMode.kCoast);
    this.m_rightTwo.setIdleMode(IdleMode.kCoast);
    this.m_rightThree.setIdleMode(IdleMode.kBrake);
  }

  public double getLeftEncPosition() {
    return this.enc_left.getPosition();
  }

  public double getLeftEncVelocity() {
    return this.enc_left.getVelocity();
  }

  public double getRightEncPosition() {
    return this.enc_right.getPosition();
  }

  public double getRightEncVelocity() {
    return this.enc_right.getVelocity();
  }

  public AHRS getgyro()
  {
    return this.gyro;
  }

  public double getgyroAngle() {
    return this.gyro.getAngle(); 
  }

  public double getgyroAxis() {
    return this.gyro.getBoardYawAxis().board_axis.getValue();
  }

  public void resetgyro() {
    this.gyro.reset();
    this.gyro.zeroYaw();
  }
}