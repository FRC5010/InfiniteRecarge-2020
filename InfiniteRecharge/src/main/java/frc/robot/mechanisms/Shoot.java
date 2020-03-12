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
import frc.robot.commands.CameraCalibrateShooter;
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
    public CANSparkMax shootMotor2;
    public CANPIDController m_pidController;
    public POVButton spinUp, baseUp;
    public POVButton spinDown, baseDown;
    public Button calButton;

    public Shoot(Joystick operator, Joystick driver, VisionSystem shooterVision) {
        this.diver = operator;
        this.shootMotor = new CANSparkMax(5, MotorType.kBrushless); 
        shootMotor2 = new CANSparkMax(13, MotorType.kBrushless);
        shootMotor.setInverted(true);
        shootMotor2.follow(shootMotor, false);
        shootMotor2.setInverted(false);
        shootMotor.setSmartCurrentLimit(40);
        calButton = new JoystickButton(driver, ControlConstants.calibrate);
        spinUp = new POVButton(operator, ControlConstants.incShooter);
        spinDown = new POVButton(operator, ControlConstants.decShooter);
        baseUp = new POVButton(driver, ControlConstants.incShooter);
        baseDown = new POVButton(driver, ControlConstants.decShooter);

        m_pidController = shootMotor.getPIDController();
        shooterMain = new ShooterMain(shootMotor, m_pidController);
        calButton.whenPressed(new CameraCalibrateShooter(shooterVision));
        spinUp.whenPressed(new InstantCommand(() -> ShooterConstants.baseSpeed+=10));
        spinDown.whenPressed(new InstantCommand(() -> ShooterConstants.baseSpeed-=10));
        baseUp.whenPressed(new InstantCommand(() -> ShooterConstants.distanceToRPM+=.1));
        baseDown.whenPressed(new InstantCommand(() -> ShooterConstants.distanceToRPM-=.1));
    }
}
