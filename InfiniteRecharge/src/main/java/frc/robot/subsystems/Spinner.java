/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spinner extends SubsystemBase {
  SpeedController spinner;
  DoubleSolenoid spinDeployment;

  /**
   * Creates a new Spinner.
   */
  public Spinner(SpeedController spinner, int fwdChannel, int revChannel) {
    this.spinner = spinner;
   // spinDeployment = new DoubleSolenoid(fwdChannel, revChannel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spin(){
    spinner.set(0.5);
  }

  public void stop(){
    spinner.set(0);
  }
}
