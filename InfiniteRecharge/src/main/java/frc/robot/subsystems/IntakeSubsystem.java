/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.mechanisms.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new IntakeSubsystem.
   */
  SpeedController intakeMotor;
  Joystick joystick;
  public IntakeSubsystem(SpeedController intakeMotor, Joystick joystick) {
    this.intakeMotor = intakeMotor;
    this.joystick = joystick;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeMotor.set(joystick.getRawAxis(IntakeConstants.intakeAxis) * IntakeConstants.maxOutput);
    
  }
  public void spinIn(){
    intakeMotor.set(IntakeConstants.maxOutput);
  }

  public void spinOut(){
    intakeMotor.set(-0.5);
  }

  public void stop(){
    intakeMotor.set(0);
  }
}
