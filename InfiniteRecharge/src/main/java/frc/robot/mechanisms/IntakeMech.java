/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ControlConstants;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Add your docs here.
 */
public class IntakeMech {
    public CANSparkMax intakeMotor;
    public IntakeSubsystem intakeMain;
    public Joystick joystick;
    public Button rightBumper;
    public DoubleSolenoid solenoid;

    public IntakeMech(Joystick joystick){
        this.joystick = joystick;
        this.rightBumper = new JoystickButton(joystick, ControlConstants.toggleIntakeButton);
        this.solenoid = new DoubleSolenoid(IntakeConstants.forwardChannel, IntakeConstants.reverseChannel);
        intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorChannel, MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(25);
        intakeMain = new IntakeSubsystem(intakeMotor, joystick, solenoid);
        rightBumper.whenPressed(new ToggleIntake(intakeMain));
    }

}
