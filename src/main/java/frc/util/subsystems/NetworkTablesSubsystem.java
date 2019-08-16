/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util.subsystems;

import edu.wpi.first.networktables.*;

public abstract class NetworkTablesSubsystem extends SensorSubsystem
{
  private static NetworkTableInstance networkTables = NetworkTableInstance.getDefault();

  public static double getDouble(String tableName, String dataName)
  {
    return networkTables.getTable(tableName).getEntry(dataName).getDouble(0.0);
  }

  @Override
  public void initDefaultCommand() {}
}