/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ArmIntakeButton;
import frc.robot.commands.ArmOuttakeButton;
import frc.robot.commands.ShiftGear;

public class OI 
{
  /*****
   * Joystick Objects
   *****/
  public final Joystick js_drive = new Joystick(Robot.ROBOTMAP.js_one);
  public final Joystick js_operate = new Joystick(Robot.ROBOTMAP.js_two);

  public OI()
  {
    Button shift = new JoystickButton(this.js_drive, Robot.ROBOTMAP.btn_shift);
    shift.whenPressed(new ShiftGear());
    shift.close();

    Button intake = new JoystickButton(this.js_operate, Robot.ROBOTMAP.btn_intake);
    intake.whenPressed(new ArmIntakeButton());
    intake.close();

    Button outtake = new JoystickButton(this.js_operate, Robot.ROBOTMAP.btn_outtake);
    outtake.whenPressed(new ArmOuttakeButton());
    outtake.close();
  }
}
