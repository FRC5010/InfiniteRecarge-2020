/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ControlConstants;

/**
 * Add your docs here.
 */
public class VisionValues {

    // the essential variables, stuff they already give me
    // TODO: If you ever get rid of a networkentry variable in opensight, update
    // vision classes
    private double centerX;
    private double centerY;
    private double area;

    // variables needed to process new variables, plus the new variables
    // angles
    static final double screenSizeX = 160;
    static final double screenSizeY = 120;
    static final double camFovX = 61;
    static final double camFovY = 34.3;
    private double angleX;
    private double angleY;
    // distance
    private double camHeight, camAngle, targetHeight;
    private double distance;
    private ShuffleboardLayout visionLayout;

    public VisionValues() {
        
    }

    public VisionValues(double centerX, double centerY, double angleX, double angleY, double distance) {
        this.centerX = centerX;
        this.centerY = centerY;
        this.angleX = angleX;
        this.angleY = angleY;
        this.distance = distance;
    }

    public VisionValues(String path, double camHeight, double camAngle, double targetHeight, int columnIndex) {
        this.camHeight = camHeight;
        this.camAngle = camAngle;
        this.targetHeight = targetHeight;
        ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay);
        visionLayout = driverTab.getLayout(path + " Vision", BuiltInLayouts.kList).withPosition(columnIndex, 0).withSize(2, 4);
        // CameraServer camera = CameraServer.getInstance();
        // VideoSink vs = camera.getVideo();
        // visionLayout.add(path + " cam", vs).withWidget(BuiltInWidgets.kCameraStream);
        visionLayout.addNumber(path + " Distance", this::getDistance).withSize(1, 1);
        visionLayout.addNumber(path + " Cam Angle", this::getCamAngle).withSize(1, 1);
        visionLayout.addNumber(path + " X Angle", this::getAngleX).withSize(1, 1);
        visionLayout.addNumber(path + " Y Angle", this::getAngleY).withSize(1, 1);
    }

    public void updateViaNetworkTable(String path) {
        // essential variables from NetworkTables
    
        centerX = VisionSystem.table.getTable(path).getEntry("center-x").getDouble(0);
        centerY = VisionSystem.table.getTable(path).getEntry("center-y").getDouble(0);
        area = 0;
        // calculating angle
        angleY = camFovY * (screenSizeY / 2 - (centerY * (screenSizeY / 2))) / screenSizeY;
        angleX = camFovX / 2 * centerX / 0.9;
       
        // calculating distance
        distance = (targetHeight - camHeight) / Math.tan(Math.toRadians(angleY+camAngle));

        SmartDashboard.putNumber(path + " centerX", centerX);
        SmartDashboard.putNumber(path + " centerY", centerY);
    }
    public void calibarateCamAngle(){
       camAngle =  Math.toDegrees(Math.atan(64/120)) -  angleY;
       
    }
    public void setCamAngle(double a ){
        camAngle = a;
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
    public double getCamAngle(){
        return camAngle;
    }
    public double getDistanceViaArea() {
        return 0;
    }
}
