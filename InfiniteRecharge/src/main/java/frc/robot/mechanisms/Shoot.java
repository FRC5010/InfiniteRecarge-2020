/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.mechanisms;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SpinShooter;
import frc.robot.subsystems.ShooterMain;

/**
 * Add your docs here.
 */


public class Shoot {
    public Joystick diver;
    public Button buttonA;
    public ShooterMain shooterMain;
    public CANSparkMax shootMotor; 
    public CANPIDController m_pidController;
    private SpinShooter command;
    public Shoot(Joystick driver){
        this.diver = driver;
        this.shootMotor = new CANSparkMax(4, MotorType.kBrushless);
        this.buttonA = new JoystickButton(driver, 1);
        m_pidController = shootMotor.getPIDController();
        m_pidController.setP(ShooterConstants.kP);
        m_pidController.setI(ShooterConstants.kI);
        m_pidController.setD(ShooterConstants.kD);
        m_pidController.setIZone(ShooterConstants.kIz);
        m_pidController.setFF(ShooterConstants.kFF);
        m_pidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
    
        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", ShooterConstants.kP);
        SmartDashboard.putNumber("I Gain", ShooterConstants.kI);
        SmartDashboard.putNumber("D Gain", ShooterConstants.kD);
        SmartDashboard.putNumber("I Zone", ShooterConstants.kIz);
        SmartDashboard.putNumber("Feed Forward",ShooterConstants.kFF);
        SmartDashboard.putNumber("Max Output", ShooterConstants.kMaxOutput);
        SmartDashboard.putNumber("Min Output", ShooterConstants.kMinOutput);
        
        
        
        shooterMain = new ShooterMain(shootMotor,m_pidController);
          
        
        buttonA.whenPressed(new SpinShooter(shooterMain));
    
    
    
    }
}
