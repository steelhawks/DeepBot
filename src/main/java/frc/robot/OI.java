/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ShiftGear;

public class OI 
{
  /*****
   * Joystick Objects
   *****/
  private final Joystick DRIVE_JOYSTICK = new Joystick(Robot.ROBOTMAP.JOYSTICK_PORT_ONE);

  public OI()
  {
    Button SHIFT_BUTTON = new JoystickButton(getDriveJoystick(), Robot.ROBOTMAP.SHIFT_BUTTON);

    SHIFT_BUTTON.whenPressed(new ShiftGear());

    SHIFT_BUTTON.close();
  }

  /*****
   * Getter methods
   *****/
  public Joystick getDriveJoystick()
  {
    return this.DRIVE_JOYSTICK;
  }
}
