/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util.subsystems;

public abstract class VisionSubsystem extends NetworkTablesSubsystem
{
  
  /** Gets the x coordinate value from Network Tables. @return The x coordinate value */
  public double getNTXPos(String name)
  {
    return getDouble("CVResultsTable", "CenterPoint X " + name);
  }

  /** Gets the y coordinate value from Network Tables. @return The y coordinate value */
  public double getNTYPos(String name)
  {
    return getDouble("CVResultsTable", "CenterPoint Y " + name);
  }

  /** Gets the apparent calculated distance between the robot and the object from Network Tables. @return The apparent calculated distance between the robot and the object */
  public double getNTDistance(String name)
  {
    return getDouble("CVResultsTable", "Distance " + name);
  }

  /** Gets the angle that the ROBOT'S line of vision and the center of the object creates from Network Tables. @return The angle that the ROBOT'S line of vision and the center of the object creates */
  public double getNTAngle(String name)
  {
    return getDouble("CVResultsTable", "Angle " + name);
  }
}