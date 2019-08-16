/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

public abstract class MechanicalSubsystem extends Subsystem
{
  /** Stops the subsystem. @return True if stopped */
  abstract public boolean stop();

  /** Pings the subsystem. */
  abstract public void ping();

  /** Checks if the subsystem is functioning properly. @return True if functioning */
  abstract public boolean isAlive();
}