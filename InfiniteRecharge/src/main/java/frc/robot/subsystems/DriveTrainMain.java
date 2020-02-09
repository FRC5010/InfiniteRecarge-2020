/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainMain extends SubsystemBase {
  /**
   * Creates a new DriveTrainMain.
   */
  private SpeedController leftMaster;
  private SpeedController rightMaster;
  private Joystick driver;
  private PowerDistributionPanel pdp;

  public DriveTrainMain(SpeedController left, SpeedController right, Joystick driver) {
    leftMaster = left;
    rightMaster = right;
    this.driver = driver;

    pdp = new PowerDistributionPanel();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arcadeDrive(scaleInputs(-driver.getRawAxis(1)), scaleInputs(driver.getRawAxis(4)));
    //System.out.println("Trying to drive from susbsystem");
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
  }

  public void arcadeDrive(double fPow, double tPow) {
    leftMaster.set(fPow + tPow);
    rightMaster.set(fPow - tPow);
  }

  public double scaleInputs(double input) {
    if (input > -.1 && input < .1) {
      return 0.0;
    }
    if (input > 1) {
      return 1;
    }
    if (input < -1) {
      return -1;
    }
    boolean speedControl = driver.getRawButton(6);
    return Math.pow(input, 3) * (!speedControl ? 1.0 : 0.3);

  }

  public void setMaxOutput(double maxOutput) {
    leftMaster.set(maxOutput);
    rightMaster.set(maxOutput);
  }
}