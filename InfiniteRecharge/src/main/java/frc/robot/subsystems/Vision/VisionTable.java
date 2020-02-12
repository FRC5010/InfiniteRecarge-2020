/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionTable extends SubsystemBase {
  
  // This class initializes and stores the tables

  NetworkTableInstance table;
  public VisionRawValues intakeCam;
  public VisionRawValues shooterCam;

  public VisionTable() {
    table = NetworkTableInstance.getDefault();
    intakeCam = new VisionRawValues(table, "intake");
    shooterCam = new VisionRawValues(table, "shooter");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeCam.printValues();
    shooterCam.printValues();
  }
}
