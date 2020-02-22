/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.mechanisms.TelescopConstants;

public class TelescopSubsystem extends SubsystemBase {
  /**
   * Creates a new TelescopSubsystem.
   */

  private CANSparkMax winch1, winch2, arm1, arm2;
  private Joystick driver;
  private Joystick operator;

  public TelescopSubsystem(CANSparkMax winch1, CANSparkMax winch2, CANSparkMax arm1, CANSparkMax arm2, Joystick driver, Joystick operator) {
    this.winch1 = winch1;
    this.winch2 = winch2;
    this.arm1 = arm1;
    this.arm2 = arm2;
    this.driver = driver;
    this.operator = operator;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinArmMotor1(){
    arm1.set(operator.getRawAxis(TelescopConstants.arm1Axis));
  }
  
  public void stopArmMotor1(){
    arm1.set(0);
  }

  public void spinArmMotor2(){
    arm2.set(operator.getRawAxis(TelescopConstants.arm2Axis));
  }

  public void stopArmMotor2(){
    arm2.set(0);
  }

  public void spinWinchMotor1(){
    winch1.set(operator.getRawAxis(TelescopConstants.winch1Axis));
  }

  public void spinWinchMotor1Reversed(){
    winch1.set(operator.getRawAxis(TelescopConstants.winch1Axis));
  }

  public void stopWinchMotor1(){
    winch1.set(0);
  }

  public void spinWinchMotor2(){
    winch2.set(operator.getRawAxis(TelescopConstants.winch2Axis));
  }

  public void spinWinchMotor2Reversed(){
    winch2.set(operator.getRawAxis(TelescopConstants.winch2Axis));
  }

  public void stopWinchMotor2(){
    winch2.set(0);
  }

  public boolean checkEncoderValue(double finalValue){
    //Currently set to only encoder one.
    // // if(climber.winchEncoder1.get() == finalValue){
    //   return true;
    // }
    return false;
  }
}
