package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.mechanisms.ClimberConstants;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
public class Climber extends SubsystemBase{
  
  private DoubleSolenoid topSolenoid;
  private DoubleSolenoid bottomSolenoid;
  //Will be changed to correct speed controller later.
  private SpeedController winch;
  private CANPIDController m_pidcontroller;
  private boolean topExteded = false;
  private boolean bottomExtended = false;
  public Climber(DoubleSolenoid topSolenoid, DoubleSolenoid bottomSolenoid, SpeedController winch, CANPIDController pidControl) {
      this.topSolenoid = topSolenoid;
      this.bottomSolenoid = bottomSolenoid;
      this.winch = winch;
      this.m_pidcontroller = pidControl;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extendTop(){
    if(topExteded == false){
      topSolenoid.set(DoubleSolenoid.Value.kForward);
      topExteded = true;
    }else{
      topSolenoid.set(DoubleSolenoid.Value.kReverse);
      topExteded = false;
    }
  }  

  public void extendBottom(){
    if(bottomExtended == false){
      bottomSolenoid.set(DoubleSolenoid.Value.kForward);
      bottomExtended = true;
    }else{
      bottomSolenoid.set(DoubleSolenoid.Value.kReverse);
      bottomExtended = false;
    }
  }

  public void spinUpWinch(){
    winch.set(ClimberConstants.winchSpeed);
  }

  public void stopWinch(){
    winch.set(0);
  }
  

}
