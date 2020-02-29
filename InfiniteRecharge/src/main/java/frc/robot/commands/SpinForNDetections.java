/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.WheelColor;

public class SpinForNDetections extends CommandBase {
  private Spinner spinner;
  private WheelColor wheel;
  private int detections;
  //Used if the position control constructor is called for.
  private boolean needGameData;
  private boolean isRotationTime;

  //This is for position control, game requires the color to be moved to the game sensor, which is two positions away from our sensor.
  private final int colorOffset = 2;
  /**
   * Creates a new SpinForNDetections.
   */

   /** 
    * Two different constructors for two different uses, check comments of each.
   */

  //Used for position control.
  public SpinForNDetections(Spinner spinner, WheelColor wheel) {
    //When testing, put the colors in the game order since colormap relies on order of the colors.
    //The direction it's spinning is clockwise.  Spinner must spin wheel clockwise in order to work correctly.
    this.spinner = spinner;
    this.wheel = wheel;
    //Basic initialization (add colorOffset to this value below in init()).
    this.detections = -1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spinner);
    addRequirements(wheel);
    needGameData = true;
    isRotationTime = false;
  }

  //Used for rotation control.
  public SpinForNDetections(Spinner spinner, WheelColor wheel, int detections) {
    this.spinner = spinner;
    this.wheel = wheel;
    this.detections = detections;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spinner);
    addRequirements(wheel);
    needGameData = false;
    isRotationTime = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Resent the color counter to count the total colors upon initialization of this command.
    wheel.setColorCount(0);
    if(needGameData){
      detections = -1;
    }   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(detections == -1 && needGameData){
      detections = wheel.determineGameData();
      if(detections > -1){
        detections += 2;
        spinner.spin(0.22);
      }
    }
    else if (isRotationTime){
      spinner.spin(0.35);
    }
    

    SmartDashboard.putNumber("Distance to Objective", detections);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spinner.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("Current Count", wheel.getTotalCount());
    return wheel.getTotalCount() >= detections;
  }
}
