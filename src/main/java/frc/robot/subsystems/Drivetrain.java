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
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.commands.DiffDrive;
import frc.robot.Robot;

public class Drivetrain extends Subsystem 
{
  //SPARK MAX LEFT MOTORS
  private final CANSparkMax LEFT_M_ONE;
  private final CANSparkMax LEFT_M_TWO;
  private final CANSparkMax LEFT_M_THREE;
  
  //SPARK MAX RIGHT MOTOR
  private final CANSparkMax RIGHT_M_ONE;
  private final CANSparkMax RIGHT_M_TWO;
  private final CANSparkMax RIGHT_M_THREE;

  //SPEED CONTROLLER GROUPS
  private final SpeedControllerGroup LEFT_M_GROUP;
  private final SpeedControllerGroup RIGHT_M_GROUP;

  //DIFFERENTIAL DRIVE
  private final DifferentialDrive DIFF_DRIVE;

  //SHIFTING SOLENOIDS
  private final DoubleSolenoid SHIFT_SOL;

  //NAVX MXP GYRO
  private final AHRS GYRO;
  private final double KP_GYRO;

  //NEO MOTOR ENCODERS
  public CANEncoder leftNeoEnc;
  public CANEncoder rightNeoEnc;

  //GRAYHILL OPTICAL ENCODERS
  public Encoder leftEnc;
  public Encoder rightEnc;

  //DRIVETRAIN CONSTRUCTOR
  public Drivetrain() 
  {
    //SPARK MAX LEFT MOTORS
    this.LEFT_M_ONE = new CANSparkMax(Robot.ROBOTMAP.LEFT_MOTOR_PORT_ONE, MotorType.kBrushless);
    this.LEFT_M_TWO = new CANSparkMax(Robot.ROBOTMAP.LEFT_MOTOR_PORT_TWO, MotorType.kBrushless);
    this.LEFT_M_THREE = new CANSparkMax(Robot.ROBOTMAP.LEFT_MOTOR_PORT_THREE, MotorType.kBrushless);
    
    //SPARK MAX RIGHT MOTORS
    this.RIGHT_M_ONE = new CANSparkMax(Robot.ROBOTMAP.RIGHT_MOTOR_PORT_ONE, MotorType.kBrushless);
    this.RIGHT_M_TWO = new CANSparkMax(Robot.ROBOTMAP.RIGHT_MOTOR_PORT_TWO, MotorType.kBrushless);
    this.RIGHT_M_THREE = new CANSparkMax(Robot.ROBOTMAP.RIGHT_MOTOR_PORT_THREE, MotorType.kBrushless);

    //SPEED CONTROLLER GROUPS
    this.LEFT_M_GROUP = new SpeedControllerGroup(this.LEFT_M_ONE, this.LEFT_M_TWO, this.LEFT_M_THREE);
    this.RIGHT_M_GROUP = new SpeedControllerGroup(this.RIGHT_M_ONE, this.RIGHT_M_TWO, this.RIGHT_M_THREE);

    //DIFFERENTIAL DRIVE
    this.DIFF_DRIVE = new DifferentialDrive(this.LEFT_M_GROUP, this.RIGHT_M_GROUP);

    //NAVX MXP GYRO
    this.GYRO = new AHRS(SPI.Port.kMXP);
    this.KP_GYRO = Robot.ROBOTMAP.KP_GYRO;

    //SHIFTING SOLENOIDS
    this.SHIFT_SOL = new DoubleSolenoid(Robot.ROBOTMAP.SHIFT_PORT_ON, Robot.ROBOTMAP.SHIFT_PORT_OFF);

    //NEO MOTOR ENCODERS
    this.leftNeoEnc = this.LEFT_M_ONE.getEncoder();
    this.rightNeoEnc = this.RIGHT_M_ONE.getEncoder();

    //GRAYHILL OPTICAL ENCODERS
    this.leftEnc = new Encoder(Robot.ROBOTMAP.LEFT_ENC_PORT_A, Robot.ROBOTMAP.LEFT_ENC_PORT_B, false, EncodingType.k4X);
    this.rightEnc = new Encoder(Robot.ROBOTMAP.RIGHT_ENC_PORT_A, Robot.ROBOTMAP.RIGHT_ENC_PORT_B, false, EncodingType.k4X);

    //SET ROBOT TO LOW GEAR
    SHIFT_SOL.set(DoubleSolenoid.Value.kReverse);

    //SET MOTORS TO BREAK
    setMotorsBrake();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DiffDrive());
  }

  //DRIVING METHOD
  public void arcadeDrive(Joystick stick) {
    this.DIFF_DRIVE.arcadeDrive(stick.getY(), -stick.getTwist());
  }

  //SHIFTING METHOD -- if { LOW } else { HIGH }
  public void shiftGear() {
    if(SHIFT_SOL.get() == DoubleSolenoid.Value.kForward) {
      this.SHIFT_SOL.set(DoubleSolenoid.Value.kReverse);
    } else {
      this.SHIFT_SOL.set(DoubleSolenoid.Value.kForward);
    }
  }

  //MOVING STRAIGHT USING THE GYRO METHOD
  public void gyroMoveStraight(double speed)
  {
    this.DIFF_DRIVE.arcadeDrive(speed, -this.GYRO.getAngle() * this.KP_GYRO);
  }

  //MOVING STRAIGHT USING GYRO AND ANGLE VALUE METHOD
  public void gyroMoveStraight(double speed, double angle)
  {
    this.DIFF_DRIVE.arcadeDrive(-speed, -angle * this.KP_GYRO);
  }

  //ROTATE ROBOT
  public void rotate(double speed)
  {
    this.LEFT_M_GROUP.set(speed);
    this.RIGHT_M_GROUP.set(speed);
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
    this.LEFT_M_ONE.setIdleMode(IdleMode.kCoast);
    this.LEFT_M_TWO.setIdleMode(IdleMode.kCoast);
    this.LEFT_M_THREE.setIdleMode(IdleMode.kCoast);
    this.RIGHT_M_ONE.setIdleMode(IdleMode.kCoast);
    this.RIGHT_M_TWO.setIdleMode(IdleMode.kCoast);
    this.RIGHT_M_THREE.setIdleMode(IdleMode.kCoast);
  }

  //SET MOTORS TO BRAKE
  public void setMotorsBrake()
  {
    this.LEFT_M_ONE.setIdleMode(IdleMode.kCoast);
    this.LEFT_M_TWO.setIdleMode(IdleMode.kCoast);
    this.LEFT_M_THREE.setIdleMode(IdleMode.kBrake);
    this.RIGHT_M_ONE.setIdleMode(IdleMode.kCoast);
    this.RIGHT_M_TWO.setIdleMode(IdleMode.kCoast);
    this.RIGHT_M_THREE.setIdleMode(IdleMode.kBrake);
  }

  public double getLeftEncRate() {
    return this.leftEnc.getRate();
  }

  public double getLeftEncDist() {
    return this.leftEnc.getDistance();
  }

  public double getRightEncRate() {
    return this.rightEnc.getRate();
  }

  public double getRightEncDist() {
    return this.rightEnc.getDistance();
  }

  public AHRS getGyro()
  {
    return this.GYRO;
  }

  public double getGyroAngle() {
    return this.GYRO.getAngle(); 
  }

  public double getGyroAxis() {
    return this.GYRO.getBoardYawAxis().board_axis.getValue();
  }

  public void resetGyro() {
    this.GYRO.reset();
    this.GYRO.zeroYaw();
  }
}