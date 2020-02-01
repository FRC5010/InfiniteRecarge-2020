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
import frc.robot.commands.LoadShaftCommand;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShaftSubsystem;

/**
 * Add your docs here.
 */
public class Intake {
    public CANSparkMax intakeMotor;
    public IntakeSubsystem intakeMain;
    public Joystick joystick;
    public Button rightBumper;
    public Button leftBumper;
    public DoubleSolenoid solenoid;

    public Intake(Joystick joystick,ShaftSubsystem shaft){
        this.joystick = joystick;
        this.rightBumper = new JoystickButton(joystick, 6);
        this.leftBumper = new JoystickButton(joystick, 5);
        this.solenoid = new DoubleSolenoid(IntakeConstants.leftPiston, IntakeConstants.rightPiston);
        intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorChannel, MotorType.kBrushless);
        intakeMain = new IntakeSubsystem(intakeMotor, joystick, solenoid);
        rightBumper.whenPressed (new ToggleIntake(intakeMain).raceWith(new LoadShaftCommand(shaft)));
    }

}
