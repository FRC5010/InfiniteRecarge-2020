/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControlConstants;
import frc.robot.mechanisms.ShooterConstants;

public class ShooterMain extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  private double setPoint = 0;
  private boolean readyToShoot = false;
  // Change to speed controller later
  private CANSparkMax controller;
  private CANPIDController m_pidController;

  public ShooterMain(CANSparkMax controller, CANPIDController pidControl) {
    this.controller = controller;
    this.m_pidController = pidControl;

    m_pidController.setP(ShooterConstants.kP);
    m_pidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Shooter P Gain", ShooterConstants.kP);
    SmartDashboard.putNumber("Shooter Max Output", ShooterConstants.kMaxOutput);
    SmartDashboard.putNumber("Shooter Min Output", ShooterConstants.kMinOutput);

    ShuffleboardLayout layout = Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay)
        .getLayout("Shooter", BuiltInLayouts.kList).withPosition(ControlConstants.shooterColumn, 1).withSize(2, 4);
    
    layout.addNumber("Velocity", controller.getEncoder()::getVelocity).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Max", 6000));
    
    layout.addNumber("Set Point", this::getSetPoint).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Max", 6000));
    
    layout.addBoolean("Ready To Shoot", this::getReadyToShoot).withWidget(BuiltInWidgets.kBooleanBox).withSize(1, 1);
    
    layout.addNumber("Distance to RPM", ShooterConstants::getDistanceToRPM);
    
    layout.addNumber("Base Speed", ShooterConstants::getBaseSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Temp", controller.getMotorTemperature());
    SmartDashboard.putNumber("Shooter Current", controller.getOutputCurrent());
  }

  public void end() {
    controller.set(0);
    setPoint = 0;
  }

  public void spinUpWheel() {
    // double accel = ((setPoint - controller.getEncoder().getVelocity()) /
    // setPoint) * 300;

    m_pidController.setFF((ShooterConstants.kS / setPoint + (ShooterConstants.kV)));

    m_pidController.setReference(setPoint, ControlType.kVelocity);
    SmartDashboard.putNumber("Shooter Applied", controller.getAppliedOutput());
    SmartDashboard.putNumber("Feed Forward", m_pidController.getFF());

    if (Math.abs(controller.getEncoder().getVelocity() - setPoint) < 75) {
      readyToShoot = true;
    } else {
      readyToShoot = false;
    }
  }

  public boolean getReadyToShoot() {
    return readyToShoot;
  }

  public double getSetPoint() {
    return setPoint;
  }

  public void setPoint(double setPoint) {
    this.setPoint = setPoint;
  }
}
