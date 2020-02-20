/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class VisionValues {

    // the essential variables, stuff they already give me
    // TODO: If you ever get rid of a networkentry variable in opensight, update vision classes
    double angleX;
    double angleY;
    double area;

    // variables needed to process new variables
    // plus the new variables
    double camHeight, camAngle, targetHeight;
    double distance;

    public VisionValues() {
    }

    public VisionValues(double centerX, double centerY, double angleX, double angleY, double distance) {
        // this.centerX = centerX;
        // this.centerY = centerY;
        this.angleX = angleX;
        this.angleY = angleY;
        this.distance = distance;
    }

    public VisionValues(double camHeight, double camAngle, double targetHeight) {
        this.camHeight = camHeight;
        this.camAngle = camAngle;
        this.targetHeight = targetHeight;
    }

    public void updateViaNetworkTable(String path) {
        // essential variables
        // centerX = VisionSystem.table.getTable(path).getEntry("center-x").getDouble(0);
        // centerX = VisionSystem.table.getTable(path).getEntry("center-y").getDouble(0);
        angleX = VisionSystem.table.getTable(path).getEntry("angle-x").getDouble(0);
        angleY = VisionSystem.table.getTable(path).getEntry("angle-y").getDouble(0);
        area = 0;
        // calculating new variables
        distance = (targetHeight - camHeight) / Math.tan(Math.toRadians(angleY));
        SmartDashboard.putNumber(path + " angle Y", angleY);
        SmartDashboard.putNumber("Distance to target ", distance);
    }

    // public double getCenterX() {
    //     return centerX;
    // }

    // public double getCenterY() {
    //     return centerY;
    // }

    public double getAngleX() {
        return angleX;
    }

    public double getAngleY() {
        return angleY;
    }

    public double getDistance() {
        return distance;
    }

    public double getDistanceViaArea() {
        return 0;
    }
}
