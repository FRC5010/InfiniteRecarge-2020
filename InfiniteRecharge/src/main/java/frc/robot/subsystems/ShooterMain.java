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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.SpinShooter;

public class ShooterMain extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

   private Button button;

   //Change to speed controller later
   private CANSparkMax controller;
   private CANPIDController pidControl;
   
  public ShooterMain(CANSparkMax controller,CANPIDController pidControl , Button button) {
    this.button =  button;
    this.controller = controller; 
    this.pidControl = pidControl;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    button.whenPressed(new SpinShooter(this));
  }


  public void spinUpWheel(double setPoint){
    pidControl.setReference(setPoint, ControlType.kVelocity);

  }

}
