/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShaftSubsystem extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  // Change to speed controller later
  private CANSparkMax barrelMotor;
  private CANPIDController m_pidController;
  private DoubleSolenoid solenoid;
  private boolean isExtended;
  private Joystick controller;
  private DigitalInput beamBreak;
  public ShaftSubsystem(CANSparkMax motor, DoubleSolenoid solenoid, DigitalInput beamBreak) {
    this.barrelMotor = motor;
    this.beamBreak = beamBreak;
    this.solenoid = solenoid;
    
  }
  public ShaftSubsystem(DigitalInput beamBreak, CANSparkMax motor, Joystick operator){
    this.beamBreak = beamBreak;
    this.barrelMotor = motor;
    this.controller = operator;
  }

  @Override
  public void periodic() {
   // spinUpShaft(controller.getRawAxis(3));
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Barrel Motor Temp", barrelMotor.getMotorTemperature());
    SmartDashboard.putNumber("Barrel Duty Cycle", barrelMotor.getAppliedOutput());
    SmartDashboard.putNumber("Barrel Output Current", barrelMotor.getOutputCurrent());
  }

  public void spinUpShaft(double speed) {
   
    barrelMotor.set(speed);
    
  }

  public void toggleShaftHeight() {
    if (isExtended) {
      lowerShaft();
    } else {
      raiseShaft();
    }
  }

  public void raiseShaft() {
    isExtended = true;
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void lowerShaft() {
    isExtended = false;
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

}
