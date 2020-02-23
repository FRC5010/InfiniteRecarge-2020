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
    private SpeedController spinnerMotor = new CANSparkMax(SpinConstants.spinnerMotorChannel, MotorType.kBrushless);
    private Solenoid spinnerSolenoid;
    private JoystickButton deployButton;

    public SpinControl(Joystick driver, Joystick operator) {
        this.driver = driver;
        this.operator = operator;
        init();
        configureButtonBindings();
    }

    public void init() {
        spinnerSolenoid = new Solenoid(4);
        spinner = new Spinner(spinnerMotor, 0, 1, spinnerSolenoid);
        wheelColor = new WheelColor();
    }

    public void configureButtonBindings() {
        rotationButton = new JoystickButton(driver, ControlConstants.rotationControl);
        positionButton = new JoystickButton(driver, ControlConstants.positionControl);
        deployButton = new JoystickButton(driver, ControlConstants.spinDeploy);
        rotationButton.whenPressed(new SpinForNDetections(spinner, wheelColor, 27));
        positionButton.whenPressed(new SpinForNDetections(spinner, wheelColor));
        deployButton.whenPressed(new ToggleSpinnerDeploy(spinner));

    }

}
