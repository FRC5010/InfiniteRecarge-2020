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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ControlConstants;
import frc.robot.commands.SpinForNDetections;
import frc.robot.commands.ToggleSpinnerDeploy;
import frc.robot.subsystems.ShaftSubsystem;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.WheelColor;

/**
 * Add your docs here.
 */
public class SpinControl {
    private Spinner spinner;
    private WheelColor wheelColor;
    private Joystick driver;
    private Joystick operator;
    private JoystickButton rotationButton;
    private JoystickButton positionButton;
    private CANSparkMax spinnerMotor;
    private Solenoid spinnerSolenoid;
    private JoystickButton deployButton;
    ShaftSubsystem shaftSubsystem;

    public SpinControl(Joystick driver, Joystick operator, ShaftSubsystem shaftSubsystem) {
        this.driver = driver;
        this.operator = operator;
        this.shaftSubsystem = shaftSubsystem;
        init();
        configureButtonBindings();
    }

    public void init() {
        spinnerMotor = new CANSparkMax(SpinConstants.spinnerMotorChannel, MotorType.kBrushless);
        //spinnerMotor.setInverted(true);
        spinnerSolenoid = new Solenoid(4);
        spinner = new Spinner(spinnerMotor, 0, 1, spinnerSolenoid, shaftSubsystem);
        wheelColor = new WheelColor();
 
        shaftSubsystem.setSpinner(spinner);
    }

    public void configureButtonBindings() {
        rotationButton = new JoystickButton(driver, ControlConstants.rotationControl);
        positionButton = new JoystickButton(driver, ControlConstants.positionControl);
        deployButton = new JoystickButton(driver, 6);
        //change rotation and position to whenPressed for when color sensor attached
        rotationButton.whenPressed(new SpinForNDetections(spinner, wheelColor, 27));
        positionButton.whenPressed(new SpinForNDetections(spinner, wheelColor));
        deployButton.whenPressed(new ToggleSpinnerDeploy(spinner));

    }

}
