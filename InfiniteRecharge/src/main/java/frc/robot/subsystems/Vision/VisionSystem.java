/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSystem extends SubsystemBase {
  
  public static NetworkTableInstance table;
  String name;

  VisionValues rawValues, smoothedValues;

  public VisionSystem(String name) {
    table = NetworkTableInstance.getDefault();
    this.name = name;
    rawValues = new VisionValues();
  }

  public VisionSystem(String name, double camHeight, double camAngle, double targetHeight) {
    table = NetworkTableInstance.getDefault();
    this.name = name;
    rawValues = new VisionValues(camHeight, camAngle, targetHeight);
  }

  @Override
  public void periodic() {
    rawValues.updateViaNetworkTable("/OpenSight/" + name);
  }

  public VisionValues getRawValues() {
    return rawValues;
  }
}
