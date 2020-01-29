package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
public class Climber extends SubsystemBase{
  
  private DoubleSolenoid topSolenoid;
  private DoubleSolenoid bottomSolenoid;
  //Will be changed to correct speed controller later.
  private SpeedController winch;

  public Climber(DoubleSolenoid topSolenoid, DoubleSolenoid bottomSolenoid, SpeedController winch) {
      this.topSolenoid = topSolenoid;
      this.bottomSolenoid = bottomSolenoid;
      this.winch = winch;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extendTop(){
    topSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractTop(){
    topSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void extendBottom(){
    bottomSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractBottom(){
    bottomSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void spinWinch(){
    
  }

  public void stopWinch(){

  }
}
