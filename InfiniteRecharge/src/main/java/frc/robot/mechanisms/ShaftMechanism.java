/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.ControlConstants;
import frc.robot.commands.LoadShaftCommand;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.ToggleShaftHeight;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShaftSubsystem;
import frc.robot.subsystems.ShaftSubsystem.ShaftState;
import frc.robot.subsystems.ShooterMain;
import frc.robot.subsystems.VisionSystem;

/**
 * Add your docs here.
 */

public class ShaftMechanism {
  public Joystick driver;

  public ShaftSubsystem shaftClimber;
  public CANSparkMax shaftMotor;
  public IntakeSubsystem intakeSubsystem;
  public ShooterMain shooterSubsystem;
  public VisionSystem visionSubsystem;

  public DoubleSolenoid shaftLifter;

  public DigitalInput beamBreakIntake;
  public DigitalInput beamBreakMiddle;
  public DigitalInput beamBreakShooter;

  // //buttons
  public Button launchButton;
  public Button heightModeToggle;
  public Button heightButton;
  public Button manualUp;
  public Button manualDown;

  public ShaftMechanism(Joystick driver, Joystick operator, IntakeSubsystem intakeSubsystem, ShooterMain shooterMain, VisionSystem visionSubsystem) {
    this.driver = driver;
    this.intakeSubsystem = intakeSubsystem;
    this.shaftMotor = new CANSparkMax(8, MotorType.kBrushless);
    shaftMotor.setSmartCurrentLimit(25);

    this.launchButton = new JoystickButton(operator, ControlConstants.launchButton);
    this.heightModeToggle = new JoystickButton(driver, ControlConstants.heightModeToggle);
    this.heightButton = new JoystickButton(operator, ControlConstants.toggleBarrelHeight);
    manualUp = new POVButton(operator, ControlConstants.barrelUp);
    manualDown = new POVButton(operator, ControlConstants.barrelDown);
    shaftLifter = new DoubleSolenoid(ShaftConstants.fwdChannel, ShaftConstants.revChannel);
    beamBreakIntake = new DigitalInput(0);
    beamBreakMiddle = new DigitalInput(1);
    beamBreakShooter = new DigitalInput(2);
    shaftClimber = new ShaftSubsystem(beamBreakIntake, beamBreakMiddle, beamBreakShooter, shaftMotor, driver,
        shaftLifter);

    heightButton.whenPressed(new ToggleShaftHeight(shaftClimber));
    heightModeToggle.whenPressed(new ParallelCommandGroup(new ToggleShaftHeight(shaftClimber),new ToggleIntake(intakeSubsystem)));
    launchButton.whileHeld(new ParallelCommandGroup(new LoadShaftCommand(shaftClimber),new SpinShooter(shooterMain, visionSubsystem)));
    manualUp.whileHeld(new FunctionalCommand(
      () -> shaftClimber.state = ShaftState.manual, 
      () -> shaftClimber.spinUpShaft(.5), 
      (intr) -> { shaftClimber.spinUpShaft(0); shaftClimber.state = ShaftState.fullStop; }, 
      () -> false, 
      shaftClimber));
    manualDown.whileHeld(new FunctionalCommand(
      () -> shaftClimber.state = ShaftState.manual, 
      () -> shaftClimber.spinUpShaft(-.5), 
      (intr) -> {shaftClimber.spinUpShaft(0); shaftClimber.state = ShaftState.fullStop;}, 
      () -> false, 
      shaftClimber));
  }

  public ShaftSubsystem getSubsystem() {
    return shaftClimber;
  }

}
