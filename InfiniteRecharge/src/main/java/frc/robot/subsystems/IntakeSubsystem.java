/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.mechanisms.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new IntakeSubsystem.
   */
  CANSparkMax intakeMotor;
  Joystick joystick;
  DoubleSolenoid solenoid;
  private boolean isExtended = false;
  private double manualInput = 0;

  public IntakeSubsystem(CANSparkMax intakeMotor, Joystick joystick, DoubleSolenoid solenoid) {
    this.intakeMotor = intakeMotor;
    this.joystick = joystick;
    this.solenoid = solenoid;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    manualInput = (joystick.getRawAxis(IntakeConstants.intakeAxis) - joystick.getRawAxis(IntakeConstants.outtakeAxis)) * IntakeConstants.maxOutput;
    intakeMotor.set(manualInput);
    SmartDashboard.putNumber("Intake Motor Temp", intakeMotor.getMotorTemperature());
    SmartDashboard.putNumber("Intake Duty Cycle", intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake Output Current", intakeMotor.getOutputCurrent());
  }

  public void toggleIntake() {
    if (isExtended) {
      retractIntake();
    } else {
      extendIntake();
    }
  }

  public void extendIntake() {
    isExtended = true;
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractIntake() {
    isExtended = false;
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void spinIn() {
    if (manualInput == 0) {
      intakeMotor.set(IntakeConstants.maxOutput);
    }
  }

  public void spinOut() {
    if (manualInput == 0) {
      intakeMotor.set(-IntakeConstants.maxOutput);
    }
  }

  public void stop() {
    intakeMotor.set(0);
  }
}
