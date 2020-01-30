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
import frc.robot.commands.ClimbShaftCommand;
import frc.robot.subsystems.ShaftClimberSubsystem;

/**
 * Add your docs here.
 */


public class ShaftMechanism {
    public Joystick driver;
    public Button buttonB;
    public ShaftClimberSubsystem shaftClimber;
    public CANSparkMax shaftMotor; 
    public CANPIDController m_pidController;
    private ClimbShaftCommand command;
    public ShaftMechanism(Joystick driver){
        this.driver = driver;
        this.shaftMotor = new CANSparkMax(8, MotorType.kBrushless);
        this.buttonB = new JoystickButton(driver, 2);
        m_pidController = shaftMotor.getPIDController();
        m_pidController.setP(ShaftConstants.kP);
        m_pidController.setI(ShaftConstants.kI);
        m_pidController.setD(ShaftConstants.kD);
        m_pidController.setIZone(ShaftConstants.kIz);
        m_pidController.setFF(ShaftConstants.kFF);
        m_pidController.setOutputRange(ShaftConstants.kMinOutput, ShaftConstants.kMaxOutput);
    
        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", ShaftConstants.kP);
        SmartDashboard.putNumber("I Gain", ShaftConstants.kI);
        SmartDashboard.putNumber("D Gain", ShaftConstants.kD);
        SmartDashboard.putNumber("I Zone", ShaftConstants.kIz);
        SmartDashboard.putNumber("Feed Forward",ShaftConstants.kFF);
        SmartDashboard.putNumber("Max Output", ShaftConstants.kMaxOutput);
        SmartDashboard.putNumber("Min Output", ShaftConstants.kMinOutput);
        
        
        
        shaftClimber = new ShaftClimberSubsystem(shaftMotor,m_pidController);
          
        
        buttonB.whenPressed(new ClimbShaftCommand(shaftClimber));
    
    
    
    }
}
