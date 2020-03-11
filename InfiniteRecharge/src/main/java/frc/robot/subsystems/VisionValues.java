/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

/**
 * Add your docs here.
 */
public class VisionValues {

    // the essential variables, stuff they already give me
    // TODO: If you ever get rid of a networkentry variable in opensight, update
    // vision classes

    private boolean valid = false;
    private double centerX = 0.0;
    private double centerY = 0.0;

    private double angleX = 0.0;
    private double angleY = 0.0;
    // distance
    private double distance = 0.0;
    
    public VisionValues() {        
    }

    public VisionValues(boolean valid, double centerX, double centerY, double angleX, double angleY, double distance) {
        this.valid = valid;
        this.centerX = centerX;
        this.centerY = centerY;
        this.angleX = angleX;
        this.angleY = angleY;
        this.distance = distance;
    }

    public boolean getValid() {
        return valid;
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
