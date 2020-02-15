/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionRawValues {

    // This class gets the current vision values from the network tables (unsmoothed, raw)

    String tableName;
    NetworkTableEntry centerX, centerY, angleX, angleY;

    public VisionRawValues(NetworkTableInstance ntTable, String tableName) {
        this.tableName = tableName;
        String tablePath = "/OpenSight/" + tableName;
        centerX = ntTable.getTable(tablePath).getEntry("center-x");
        centerY = ntTable.getTable(tablePath).getEntry("center-y");
        angleX = ntTable.getTable(tablePath).getEntry("angle-x");
        angleY = ntTable.getTable(tablePath).getEntry("angle-y");
    }

    public void printValues() {
        SmartDashboard.putNumber(tableName + ": centerX", getCenterX());
        SmartDashboard.putNumber(tableName + ": centerY", getCenterY());
        SmartDashboard.putNumber(tableName + ": angleX", getAngleX());
        SmartDashboard.putNumber(tableName + ": angleY", getAngleY());
    }

    public double getCenterX() {
        return centerX.getDouble(0);
    }

    public double getCenterY() {
        return centerY.getDouble(0);
    }

    public double getAngleX() {
        return angleX.getDouble(0);
    }

    public double getAngleY() {
        return angleY.getDouble(0);
    }
}
