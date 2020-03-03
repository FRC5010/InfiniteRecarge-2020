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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ControlConstants;
import frc.robot.commands.LoadShaftCommand;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.ToggleLedRing;
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

  public Solenoid ledRing;

  public DigitalInput beamBreakIntake;
  public DigitalInput beamBreakMiddle;
  public DigitalInput beamBreakShooter;

  // //buttons
  public Button launchButton;
  public Button lowGoalButton;
  public Button heightModeToggle;
  public Button heightButton;
  public Button manualUp;
  public Button manualDown;
  public Button toggleLed;

  public ShaftMechanism(Joystick driver, Joystick operator, IntakeSubsystem intakeSubsystem, ShooterMain shooterMain, VisionSystem visionSubsystem) {
    this.driver = driver;
    this.intakeSubsystem = intakeSubsystem;
    this.shaftMotor = new CANSparkMax(8, MotorType.kBrushless);
    shaftMotor.setSmartCurrentLimit(25);

    this.launchButton = new JoystickButton(operator, ControlConstants.launchButton);
    this.heightButton = new JoystickButton(driver, ControlConstants.heightModeToggle);
    lowGoalButton = new JoystickButton(operator, ControlConstants.lowGoalShoot);
    manualUp = new JoystickButton(operator, ControlConstants.barrelUp);
    manualDown = new JoystickButton(operator, ControlConstants.barrelDown);
    shaftLifter = new DoubleSolenoid(ShaftConstants.fwdChannel, ShaftConstants.revChannel);
    this.toggleLed = new JoystickButton(driver, ControlConstants.toggleLed);

    ledRing = new Solenoid(5); 

    beamBreakIntake = new DigitalInput(0);
    beamBreakMiddle = new DigitalInput(1);
    beamBreakShooter = new DigitalInput(2);

     shaftClimber = new ShaftSubsystem(beamBreakIntake, beamBreakMiddle, beamBreakShooter, shaftMotor, driver,shaftLifter, ledRing);
     
    toggleLed.whenPressed(new ToggleLedRing(shaftClimber));
    heightButton.whenPressed(new ToggleShaftHeight(shaftClimber, shooterMain));
    launchButton.whileHeld(new ParallelCommandGroup(new LoadShaftCommand(shaftClimber),new SpinShooter(shooterMain, visionSubsystem)));
    lowGoalButton.whileHeld(new SpinShooter(shooterMain, visionSubsystem, 1000));
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
