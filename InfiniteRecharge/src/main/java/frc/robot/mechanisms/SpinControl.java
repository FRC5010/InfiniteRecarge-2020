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
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SpinForNDetections;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.WheelColor;

/**
 * Add your docs here.
 */
public class SpinControl {
    private Spinner spinner;
    private WheelColor wheelColor;
    private Joystick driver;
    private JoystickButton rotationButton;
    private JoystickButton positionButton;
    private SpeedController spinnerMotor = new CANSparkMax(SpinConstants.spinnerMotorChannel, MotorType.kBrushless);

    public SpinControl(Joystick driver) {
        this.driver = driver;
        init();
        configureButtonBindings();
    }

    public void init() {
        spinner = new Spinner(spinnerMotor, 0, 1);
        wheelColor = new WheelColor();
    }

    public void configureButtonBindings() {
        rotationButton = new JoystickButton(driver, 1);
        positionButton = new JoystickButton(driver, 2);
        rotationButton.whenPressed(new SpinForNDetections(spinner, wheelColor, 2));
        positionButton.whenPressed(new SpinForNDetections(spinner, wheelColor));
    }
}
