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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControlConstants;
import frc.robot.commands.IntakeBalls;
import frc.robot.mechanisms.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new IntakeSubsystem.
   */
  CANSparkMax intakeMotor;
  Joystick joystick;
  DoubleSolenoid solenoid;
  private boolean isExtended = false;
  private Timer timer;

  public IntakeSubsystem(CANSparkMax intakeMotor, Joystick joystick, DoubleSolenoid solenoid) {
    this.intakeMotor = intakeMotor;
    this.joystick = joystick;
    this.solenoid = solenoid;
    timer = new Timer();
    setDefaultCommand(new IntakeBalls(this, joystick));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Motor Temp", intakeMotor.getMotorTemperature());
    SmartDashboard.putNumber("Intake Duty Cycle", intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake Output Current", intakeMotor.getOutputCurrent());
    
    // if(timer.get() < 3 && !isExtended){
    //   intakeMotor.set(-0.05);
    // }else{
    //   timer.stop();
    //   timer.reset();
    // }
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
   // timer.start();
  }

  public void spin(double power) {
    intakeMotor.set(IntakeConstants.maxOutput * power);
  }

  public void stop() {
    intakeMotor.set(0);
  }
}
