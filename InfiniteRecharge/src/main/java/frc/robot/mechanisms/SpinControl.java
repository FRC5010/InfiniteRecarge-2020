/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.mechanisms;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.WheelColor;

/**
 * Add your docs here.
 */
public class SpinControl {
    Spinner spinner;
    WheelColor wheelColor;
    SpeedController spinnerMotor = new WPI_VictorSPX(SpinConstants.spinnerMotorChannel);

    public SpinControl() {
        init();
        configureButtonBindings();
    }

    public void init() {
        spinner = new Spinner(spinnerMotor, 0, 1);
        wheelColor = new WheelColor();
    }

    public void configureButtonBindings() {

    }
}
