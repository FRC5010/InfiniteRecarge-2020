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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.LoadShaftCommand;
import frc.robot.commands.ToggleShaftHeight;
import frc.robot.subsystems.ShaftSubsystem;

/**
 * Add your docs here.
 */

public class ShaftMechanism {
  public Joystick driver;

  public ShaftSubsystem shaftClimber;
  public CANSparkMax shaftMotor;

  public DoubleSolenoid shaftLifter;

  public DigitalInput beamBreakIntake;
  public DigitalInput beamBreakMiddle;
  public DigitalInput beamBreakShooter;

  // //buttons
  public Button buttonB;
  public Button driverLB;
  public Button povUp;
  public Button povDown;

  public ShaftMechanism(Joystick driver, Joystick operator) {
    this.driver = driver;

    this.shaftMotor = new CANSparkMax(8, MotorType.kBrushless);
    shaftMotor.setSmartCurrentLimit(25);

    this.buttonB = new JoystickButton(operator, 2);
    this.driverLB = new JoystickButton(driver, 5);
    povUp = new POVButton(operator, 0);
    povDown = new POVButton(operator, 180);
    shaftLifter = new DoubleSolenoid(ShaftConstants.fwdChannel, ShaftConstants.revChannel);
    beamBreakIntake = new DigitalInput(0);
    beamBreakMiddle = new DigitalInput(1);
    beamBreakShooter = new DigitalInput(2);
    shaftClimber = new ShaftSubsystem(beamBreakIntake, beamBreakMiddle, beamBreakShooter, shaftMotor, driver,
        shaftLifter);

    driverLB.whenPressed(new ToggleShaftHeight(shaftClimber));
    buttonB.whileHeld(new LoadShaftCommand(shaftClimber));
    povUp.whenPressed(new FunctionalCommand(()->{}, () -> shaftClimber.spinUpShaft(.5), (intr) -> shaftClimber.spinUpShaft(0), () -> false, shaftClimber));
    povDown.whenPressed(new FunctionalCommand(()->{}, () -> shaftClimber.spinUpShaft(-.5), (intr) -> shaftClimber.spinUpShaft(0), () -> false, shaftClimber));

  }

  public ShaftSubsystem getSubsystem() {
    return shaftClimber;
  }

}
