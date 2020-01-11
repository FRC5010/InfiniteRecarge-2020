/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriveTrainMain extends SubsystemBase {
  /**
   * Creates a new DriveTrainMain.
   */
  private static CANSparkMax leftMaster;
  private static CANSparkMax rightMaster;
  private static Joystick driver;
  public DriveTrainMain() {
    leftMaster = RobotContainer.lDrive1;
    rightMaster = RobotContainer.rDrive1;
    driver = RobotContainer.driver;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arcadeDrive(scaleInputs(driver.getRawAxis(1
    )),scaleInputs(driver.getRawAxis(4)));
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(-rightVolts);
    }

  public void arcadeDrive(double fPow, double tPow){
    leftMaster.set(fPow+tPow);
    rightMaster.set(fPow-tPow);
  }

  public double scaleInputs(double input) {
    if(input>-.1&&input<.1){
      return 0.0;
    }
    if(input>1){
      return 1;
    }
    if(input<-1){
      return -1;
    }
    return input;
  }

public static void setMaxOutput(double maxOutput){
  leftMaster.set(maxOutput);
  rightMaster.set(maxOutput);
}
}
