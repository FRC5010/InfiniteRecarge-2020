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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.ControlConstants;
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
    public POVButton spinUp;
    public POVButton spinDown;
    public Button calButton;

    public Shoot(Joystick operator, VisionSystem shooterVision) {
        this.diver = operator;
        this.shootMotor = new CANSparkMax(5, MotorType.kBrushless); 
        shootMotor.setInverted(true);
        shootMotor.setSmartCurrentLimit(40);
        calButton = new JoystickButton(operator, ControlConstants.calibrate);
        spinUp = new POVButton(operator, ControlConstants.incShooter);
        spinDown = new POVButton(operator, ControlConstants.decShooter);

        m_pidController = shootMotor.getPIDController();
        shooterMain = new ShooterMain(shootMotor, m_pidController);

        spinUp.whenPressed(new InstantCommand(() -> ShooterConstants.baseSpeed++));
        spinDown.whenPressed(new InstantCommand(() -> ShooterConstants.baseSpeed--));
    }
}
