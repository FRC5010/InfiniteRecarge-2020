/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShaftSubsystem;
import frc.robot.subsystems.ShaftSubsystem.ShaftState;

public class LoadShaftCommand extends CommandBase {
  /**
   * Creates a new ShaftClimberSubsystem.
   */
  ShaftSubsystem shaftClimber;
  int numShoot = 0;

  public LoadShaftCommand(ShaftSubsystem shaftClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shaftClimber = shaftClimber;
    addRequirements(shaftClimber);
  }
  public LoadShaftCommand(ShaftSubsystem shaftClimber, int numShoot){
    this.shaftClimber = shaftClimber;
    this.numShoot = numShoot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shaftClimber.state = ShaftState.shootWait;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shaftClimber.spinUpShaft(0);
    shaftClimber.state = ShaftState.fullStop;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shaftClimber.getShotCount() == numShoot && numShoot !=0;
  }
}
