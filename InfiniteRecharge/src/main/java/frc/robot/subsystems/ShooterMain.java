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

  

   //Change to speed controller later
   private CANSparkMax controller;
   private CANPIDController m_pidController;
   
  public ShooterMain(CANSparkMax controller,CANPIDController pidControl ) {
  
    this.controller = controller; 
    this.m_pidController = pidControl;


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   
  }


  public void spinUpWheel(double setPoint){
    m_pidController.setReference(setPoint, ControlType.kVelocity);

  }

}
