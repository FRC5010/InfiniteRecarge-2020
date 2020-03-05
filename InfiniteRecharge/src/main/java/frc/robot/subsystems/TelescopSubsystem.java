/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControlConstants;
import frc.robot.mechanisms.TelescopConstants;

public class TelescopSubsystem extends SubsystemBase {
  /**
   * Creates a new TelescopSubsystem.
   */

  private CANSparkMax winch1, winch2, arm1, arm2;
  private Joystick driver;
  private Joystick operator;
  public CANEncoder winchEncoder1;
  public CANEncoder winchEncoder2;
  public CANEncoder armEncoder1;
  public CANEncoder armEncoder2;
  public Timer climbTimer = new Timer();
  private ShuffleboardTab climberTab;
  private boolean limitApplied = false;
  public boolean overrideArms = false;

  public TelescopSubsystem(CANSparkMax winch1, CANSparkMax winch2, CANSparkMax arm1, CANSparkMax arm2, Joystick driver,
      Joystick operator) {
    this.winch1 = winch1;
    this.winch2 = winch2;
    this.arm1 = arm1;
    this.arm2 = arm2;
    this.driver = driver;
    this.operator = operator;
    winchEncoder1 = winch1.getEncoder();

    winchEncoder2 = winch2.getEncoder();
    armEncoder1 = arm1.getEncoder();
    armEncoder2 = arm2.getEncoder();
    climbTimer.reset();
    climberTab = Shuffleboard.getTab("Climber");
    ShuffleboardLayout climberLayout = climberTab.getLayout("Climber", BuiltInLayouts.kList);
    ShuffleboardLayout arm1Layout = climberTab.getLayout("Deploy 1", BuiltInLayouts.kList);
    ShuffleboardLayout arm2Layout = climberTab.getLayout("Deploy 2", BuiltInLayouts.kList);
    ShuffleboardLayout winch1Layout = climberTab.getLayout("Winch 1", BuiltInLayouts.kList);
    ShuffleboardLayout winch2Layout = climberTab.getLayout("Winch 2", BuiltInLayouts.kList);
    climberLayout.addNumber("Climb Timer", climbTimer::get);
    climberLayout.addBoolean("Override",()-> overrideArms);

    arm1Layout.addNumber("Encoder", armEncoder1::getPosition);
    arm1Layout.addNumber("Current", arm1::getOutputCurrent);
    arm1Layout.addNumber("Output", arm1::getAppliedOutput);

    arm2Layout.addNumber("Encoder", armEncoder2::getPosition);
    arm2Layout.addNumber("Current", arm2::getOutputCurrent);
    arm2Layout.addNumber("Output", arm2::getAppliedOutput);

    winch1Layout.addNumber("Encoder", winchEncoder1::getPosition);
    winch1Layout.addNumber("Current", winch1::getOutputCurrent);
    winch1Layout.addNumber("Output", winch1::getAppliedOutput);

    winch2Layout.addNumber("Encoder", winchEncoder2::getPosition);
    winch2Layout.addNumber("Current", winch2::getOutputCurrent);
    winch2Layout.addNumber("Output", winch2::getAppliedOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinArmMotors() {
    double speed = TelescopConstants.armSpeed * operator.getRawAxis(ControlConstants.combinedArmDeploy);
    if (overrideArms) {
      arm1.set(TelescopConstants.armSpeed * operator.getRawAxis(ControlConstants.leftArmDeploy));
      arm2.set(TelescopConstants.armSpeed * operator.getRawAxis(ControlConstants.combinedArmDeploy));
    } else if (armEncoder1.getPosition() > (-230.) && armEncoder2.getPosition() > (-230.)) {
      arm1.set(speed);
      arm2.set(speed);
    } else {
      arm1.set(0);
      arm2.set(0);
    }
    if (speed > 0) {
      climbTimer.start();
    } else {
      climbTimer.stop();
    }
    if (climbTimer.get() > TelescopConstants.raisedTimeLimit) {
      if (!limitApplied) {
        limitApplied = true;
        arm1.setSmartCurrentLimit(TelescopConstants.raisedCurrentLimit);
        arm2.setSmartCurrentLimit(TelescopConstants.raisedCurrentLimit);
      }
    }
  }

  public void stopArmMotors() {
    arm1.set(0);
    arm2.set(0);
  }

  public void spinWinchMotors() {
    double speed = TelescopConstants.winchSpeed * Math.abs(driver.getRawAxis(ControlConstants.winch1Axis));
    winch1.set(speed);
    winch2.set(speed);
  }

  public void stopWinchMotors() {
    winch1.set(0);
    winch2.set(0);
    climbTimer.stop();
  }

  public boolean checkEncoderValue(double finalValue) {
    // Currently set to only encoder one.
    if (winchEncoder1.getPosition() >= finalValue) {
      return true;
    }
    return false;
  }
}
