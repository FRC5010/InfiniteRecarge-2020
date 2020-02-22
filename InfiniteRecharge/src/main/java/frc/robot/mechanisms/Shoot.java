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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ChangeSpeed;
import frc.robot.commands.SpinShooter;
import frc.robot.subsystems.ShooterMain;
import frc.robot.subsystems.VisionSystem;

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
    public POVButton spinUp;
    public POVButton spinDown;
    public Shoot(Joystick operator, VisionSystem shooterVision){
        this.diver = operator;
        this.shootMotor = new CANSparkMax(5, MotorType.kBrushless); //This needs to be changed to 5
        shootMotor.setInverted(true);
       shootMotor.setSmartCurrentLimit(40);

        spinUp = new  POVButton(operator, 0);

        spinDown = new POVButton(operator, 90);
        m_pidController = shootMotor.getPIDController();
       
        
        
        
        shooterMain = new ShooterMain(shootMotor,m_pidController);
          
        spinUp.whenPressed(new ChangeSpeed(shooterMain, true));
        spinDown.whenPressed(new ChangeSpeed(shooterMain, false));
        
        
    
    
    
    }
}
