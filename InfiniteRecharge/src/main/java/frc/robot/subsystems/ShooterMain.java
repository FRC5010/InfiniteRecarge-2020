/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.SpinShooter;
import frc.robot.mechanisms.ShooterConstants;

public class ShooterMain extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  
  public double setPoint = 2000;

   //Change to speed controller later
   private CANSparkMax controller;
   private CANPIDController m_pidController;
   
  public ShooterMain(CANSparkMax controller,CANPIDController pidControl ) {  
    this.controller = controller; 
    this.m_pidController = pidControl;

    m_pidController.setP(ShooterConstants.kP);
    //m_pidController.setFF((1./4600)*1.25);
    m_pidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", ShooterConstants.kP);
    SmartDashboard.putNumber("Feed Forward",m_pidController.getFF());
    SmartDashboard.putNumber("Max Output", ShooterConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", ShooterConstants.kMinOutput);


  }

  @Override
  public void periodic() {
    setPoint = SmartDashboard.getNumber("set point", setPoint);
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter motor temp", controller.getMotorTemperature());
    SmartDashboard.putNumber("current output", controller.getOutputCurrent());
    SmartDashboard.putNumber("velocity wheel", controller.getEncoder().getVelocity()*2);

  }

  public void end(){
    m_pidController.setReference(0,ControlType.kVelocity);
  }


  public void spinUpWheel(){
    double accel = ((setPoint - controller.getEncoder().getVelocity()) / setPoint) * 300;
    
    m_pidController.setFF((ShooterConstants.kS/setPoint + (ShooterConstants.kV)));

   
   
   
    m_pidController.setReference(setPoint,ControlType.kVelocity);
    SmartDashboard.putNumber("applied output", controller.getAppliedOutput());
    SmartDashboard.putNumber("kf", m_pidController.getFF());
    SmartDashboard.putNumber("kp", m_pidController.getP());
    SmartDashboard.putNumber("velocity",controller.getEncoder().getVelocity());
  // controller.set(-.50);

  }

}

