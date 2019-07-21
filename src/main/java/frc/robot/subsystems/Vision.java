/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

public class Vision extends Subsystem
{
    private NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
    private NetworkTable networkTablesData = networkTables.getTable("CVResultsTable");
    private boolean initAlign;
    private boolean alignAngle;
    private double angle;
    private double xPosLeftLimit;
    private double xPosRightLimit;

    @Override
    public void initDefaultCommand() {}

    //Aligns the robot
    public void align()
    {
        if (Robot.OI.js_drive.getRawButtonPressed(2))
        {
            end();
        }
        else if (!this.initAlign)
        {
            System.out.println("Aligning");
            if (Math.abs(Robot.DRIVETRAIN.gyro.getAngle()) < (getAngle() - 0.1) && getXPos() < getXPosLeftLimit())
            {
                Robot.DRIVETRAIN.rotate(0.325);
            }
            else if(Math.abs(Robot.DRIVETRAIN.gyro.getAngle()) < (getAngle() - 0.1) && getXPos() > getXPosRightLimit()) 
            {
			    Robot.DRIVETRAIN.rotate(-0.325);
            }
            else 
            {
			    Robot.DRIVETRAIN.resetGyro();
                Robot.DRIVETRAIN.stop();
                System.out.println("Aligned!");
                this.initAlign = true;

            }
        }
        else
        {
            if (!Robot.SENSORS.isClose())
            {
                Robot.DRIVETRAIN.gyroMoveStraight(Robot.DRIVETRAIN.decimalSpeed(Robot.SENSORS.ULTRA.getRangeInches()));
            }
            else
            {
                Robot.DRIVETRAIN.stop();
                System.out.println("Distanced!");
                alignAngle = true;
            }
        } 
    }

    /**
     * Gets the x coordinate value of the center of the tape pair from Network Tables.
     * 
     * @return The x coordinate value of the center of the tape pair
     */
    public double getXPos()
    {
        System.out.println("Center X Tape: " + networkTablesData.getEntry("CenterPoint X Tape").getDouble(0.0));
        return networkTablesData.getEntry("CenterPoint X Tape").getDouble(0.0);
    }

    /**
     * Gets the y coordinate value of the center of the tape pair from Network Tables.
     * 
     * @return The y coordinate value of the center of the tape pair
     */
    public double getYPos()
    {
        System.out.println("Center Y Tape: " + networkTablesData.getEntry("CenterPoint Y Tape").getDouble(0.0));
        return networkTablesData.getEntry("CenterPoint Y Tape").getDouble(0.0);
    }

    /**
     * Gets the apparent calculated distance between the robot and the center of the tape pair from Network Tables.
     * 
     * @return The apparent calculated distance between the robot and the center of the tape pair
     */
    public double getDistance()
    {
        System.out.println("Distance Tape: " + networkTablesData.getEntry("Distance Tape").getDouble(0.0));
        return networkTablesData.getEntry("Distance Tape").getDouble(0.0);
    }

    /**
     * Gets the angle that the ROBOT'S line of vision and the center of the tape pair creates from Network Tables.
     * 
     * @return The angle that the ROBOT'S line of vision and the center of the tape pair creates
     */
    public double getNTAngle()
    {
        System.out.println("NT Angle Tape: " + networkTablesData.getEntry("Angle Tape").getDouble(0.0));
        return networkTablesData.getEntry("Angle Tape").getDouble(0.0);
    }

    /**
     * Sets the target angle for alignment.
     * 
     * @param double angle
     * @return void
     */
    public void setAngle(double angle)
    {
       this.angle = angle;
    }

    /**
     * Gets the target angle for alignment.
     * 
     * @return The target angle for alignment
     */
    public double getAngle()
    {
        //System.out.println(this.angle);
        return this.angle;
    }

    /**
     * Sets the x cooridnate of the left border of the center of the tape pair.
     * 
     * @param double x limit
     * @return void
     */
    public void setXPosLeftLimit(double xPosLeftLimit)
    {
       this.xPosLeftLimit = xPosLeftLimit;
    }

    /**
     * Sets the x cooridnate of the right border of the center of the tape pair.
     * 
     * @param double x limit
     * @return void
     */
    public void setXPosRightLimit(double xPosRightLimit)
    {
       this.xPosRightLimit = xPosRightLimit;
    }

    /**
     * Gets the x cooridnate of the left border of the center of the tape pair.
     * 
     * @return The x cooridnate of the left border of the tape pair
     */
    public double getXPosLeftLimit()
    {
        return this.xPosLeftLimit;
    }
   
    /**
     * Gets the x cooridnate of the right border of the center of the tape pair.
     * 
     * @return The x cooridnate of the right border of the tape pair
     */
    public double getXPosRightLimit()
    {
       return this.xPosRightLimit;
    }

    /**
     * Returns the difference between the right or left border and center of the tape pair. The difference is the smallest between the two.
     * 
     * @param x position
     * @return The difference between the right or left border and center of the tape pair
     */
    public double getXPosDiff(double xPos)
    {
        if (xPos > 160)
        {
            return xPos - 160;
        }
        return Math.abs(160 - xPos);
    }

    /**
     * Returns true if alignment and distancing to the center of the tape pair is complete.
     * 
     * @return If alignment and distancing to the center of the tape pair is complete
     */
    public boolean isAligned()
    {
        return this.alignAngle;
    }

    /**
     * Stops alignment.
     */
    public void end()
    {
        Robot.DRIVETRAIN.stop();
        System.out.println("Quit...");
        this.alignAngle = true;
    }

    /**
     * Resets alignment booleans.
     */
    public void reset()
    {
        alignAngle = false;
        initAlign = false;
    }

    public void test()
    {
        getNTAngle();
        getXPos();
        getYPos();
        getDistance();
    }
}