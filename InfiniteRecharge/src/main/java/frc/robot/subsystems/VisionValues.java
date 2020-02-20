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
    double centerX;
    double centerY;
    double area;

    // variables needed to process new variables, plus the new variables
    // angles
    double screenSizeX, screenSizeY, camFovX, camFovY;
    double angleX;
    double angleY;
    // distance
    double camHeight, camAngle, targetHeight;
    double distance;

    public VisionValues() {
    }

    public VisionValues(double centerX, double centerY, double angleX, double angleY, double distance) {
        this.centerX = centerX;
        this.centerY = centerY;
        this.angleX = angleX;
        this.angleY = angleY;
        this.distance = distance;
    }

    public VisionValues(double camHeight, double camAngle, double targetHeight) {
        this.camHeight = camHeight;
        this.camAngle = camAngle;
        this.targetHeight = targetHeight;
        this.screenSizeX = 160;
        this.screenSizeY = 120;
        this.camFovX = 61;
        this.camFovY = 34.3;
    }

    public void updateViaNetworkTable(String path) {
        // essential variables from NetworkTables
        centerX = VisionSystem.table.getTable(path).getEntry("center-x").getDouble(0);
        centerY = VisionSystem.table.getTable(path).getEntry("center-y").getDouble(0);
        area = 0;
        // calculating angle
        angleY = camFovY * (screenSizeY / 2 - centerY) / screenSizeY;
        angleX = camFovX * (screenSizeX / 2 - centerX) / screenSizeX;
        // calculating distance
        distance = (targetHeight - camHeight) / Math.tan(Math.toRadians(angleY));

        SmartDashboard.putNumber(path + " centerX", centerX);
        SmartDashboard.putNumber(path + " centerY", centerY);
        SmartDashboard.putNumber(path + " angleX", angleX);
        SmartDashboard.putNumber(path + " angleY", angleY);
    }

    public double getCenterX() {
        return centerX;
    }

    public double getCenterY() {
        return centerY;
    }

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
