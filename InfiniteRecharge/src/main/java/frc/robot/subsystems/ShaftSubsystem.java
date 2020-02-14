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
import edu.wpi.first.wpilibj.Timer;

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
  private DigitalInput bb1;
  private DigitalInput bb2;
  private DigitalInput bb3;
  private boolean isRunning = false;
  private Timer timer;
  public enum ShaftState {
    fullStop, runningClear, indexing, shooting

  }
  public ShaftState state = ShaftState.fullStop;

  public ShaftSubsystem(CANSparkMax motor, DoubleSolenoid solenoid, DigitalInput beamBreak) {
    this.barrelMotor = motor;
    //this.beamBreak = beamBreak;
    this.solenoid = solenoid;

  }

  public ShaftSubsystem(DigitalInput bb1, DigitalInput bb2, DigitalInput bb3, CANSparkMax motor, Joystick operator) {
    this.bb1 = bb1;
    this.bb2 = bb2;
    this.bb3 = bb3;
    this.barrelMotor = motor;
    this.controller = operator;
    timer = new Timer();

  }

  

  @Override
  public void periodic() {
    // spinUpShaft(controller.getRawAxis(3));
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Barrel Motor Temp", barrelMotor.getMotorTemperature());
    SmartDashboard.putNumber("Barrel Duty Cycle", barrelMotor.getAppliedOutput());
    SmartDashboard.putNumber("Barrel Output Current", barrelMotor.getOutputCurrent());
    
    if(!bb1.get() && !isRunning){
    spinUpShaft(.25);
    timer.start();
    isRunning = true;
    }
    if ((timer.get() > 0.5 && isRunning)||!bb3.get()) {
    timer.stop();
    timer.reset();
    spinUpShaft(0);
    isRunning = false;
  }


  if(state == ShaftState.shooting){
spinUpShaft(.25);
  }

  SmartDashboard.putBoolean("bb1", bb1.get());
  SmartDashboard.putBoolean("bb2", bb2.get());
  SmartDashboard.putBoolean("bb3", bb3.get());
    // if (!bb1.get() && state == ShaftState.fullStop) {
    //   // if (!bb3.get()) {
    //   //   state = ShaftState.fullStop;
    //   //   spinUpShaft(0);
    //   // } else {
    //     spinUpShaft(.25);
    //     if (!bb2.get()) {
    //       state = ShaftState.indexing;

    //     } else {
    //       state = ShaftState.runningClear;

    //     }

      
    // }

    // if (state == ShaftState.indexing && bb2.get()) {
    //   state = ShaftState.runningClear;
    // }
    // if (state == ShaftState.runningClear && (!bb2.get() || !bb3.get())) {
    //   spinUpShaft(0);
    //   state = ShaftState.fullStop;

    // }
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
