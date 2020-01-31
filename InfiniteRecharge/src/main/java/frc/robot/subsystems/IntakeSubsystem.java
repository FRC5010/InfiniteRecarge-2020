/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new IntakeSubsystem.
   */
  SpeedController intakeMotor;
  public IntakeSubsystem(SpeedController intakeMotor) {
    this.intakeMotor = intakeMotor;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void spinIn(){
    intakeMotor.set(0.5);
  }

  public void spinOut(){
    intakeMotor.set(-0.5);
  }

  public void stop(){
    intakeMotor.set(0);
  }
}
