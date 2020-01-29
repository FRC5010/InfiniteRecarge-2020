/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Add your docs here.
 */
public class Intake {
    public CANSparkMax intakeMotor;
    public IntakeSubsystem intakeMain;
    public Joystick driver;
    public Button rightBumper;
    public Button leftBumper;

    public Intake(Joystick driver){
        this.driver = driver;
        this.rightBumper = new JoystickButton(driver, 6);
        this.leftBumper = new JoystickButton(driver, 5);
        intakeMotor = new CANSparkMax(9, MotorType.kBrushless);
        intakeMain = new IntakeSubsystem(intakeMotor);
    }
}
