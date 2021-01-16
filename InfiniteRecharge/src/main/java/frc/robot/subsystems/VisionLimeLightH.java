/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

public class VisionLimeLightH extends VisionSystem {
  /**
   * Creates a new LimeLightVision.
   */
  //class for when the limelight is mounted "normally" or where the leds are mounted horizontally
  public VisionLimeLightH(String name, int colIndex) {
    super(name, colIndex);
  }

  public VisionLimeLightH(String name, double camHeight, double camAngle, double targetHeight, int colIndex) {
    super(name, camHeight, camAngle, targetHeight, colIndex);
  }

  @Override
  public void periodic() {
    updateViaNetworkTable(name);
  }

  public void updateViaNetworkTable(String path) {
    // essential variables from NetworkTables
    boolean valid = table.getTable(path).getEntry("tv").getDouble(0) == 1.0;
    
    if (valid) {
      double angleX = table.getTable(path).getEntry("tx").getDouble(0);
      double angleY = table.getTable(path).getEntry("ty").getDouble(0);
      double area = table.getTable(path).getEntry("ta").getDouble(0);

      // calculating distance
      double distance = (targetHeight - camHeight) / Math.tan(Math.toRadians(angleY + camAngle));
      rawValues = new VisionValues(valid, 0, 0, angleX, angleY, distance);
    } else {
      rawValues = new VisionValues();
    }
  }

  public void calibarateCamAngle(double angleY) {
    camAngle = 30.345 - angleY;
  }

  public void setLight(boolean on) {
    table.getTable(name).getEntry("ledMode").setNumber(on ? 3 : 1);
  }

  public void flashLight() {
    table.getTable(name).getEntry("ledMode").setNumber(2);
  }

  public boolean isLightOn() {
    return 1 != table.getTable(name).getEntry("ledMode").getNumber(0).intValue();
  }
}
