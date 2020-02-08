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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShaftSubsystem extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  // Change to speed controller later
  private CANSparkMax controller;
  private CANPIDController m_pidController;
  private DoubleSolenoid solenoid;
  private boolean isExtended;
  private Joystick speed;
  private DigitalInput beamBreak;
  public ShaftSubsystem(CANSparkMax controller, DoubleSolenoid solenoid, DigitalInput beamBreak) {
    this.controller = controller;
    this.beamBreak = beamBreak;
    this.solenoid = solenoid;
    
  }
  public ShaftSubsystem(DigitalInput beamBreak, CANSparkMax controller, Joystick operator){
    this.beamBreak = beamBreak;
    this.controller = controller;
    this.speed = operator;
  }

  @Override
  public void periodic() {
    spinUpShaft(speed.getRawAxis(3));
    // This method will be called once per scheduler run
   // System.out.println(beamBreak.get());
  }

  public void spinUpShaft(double speed) {
   
    controller.set(speed);
    
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
