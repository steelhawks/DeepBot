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
import frc.robot.commands.AlignTape;
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

    Button align = new JoystickButton(this.js_drive, Robot.ROBOTMAP.btn_align);
    align.whenPressed(new AlignTape());
    align.close();
  }
}
