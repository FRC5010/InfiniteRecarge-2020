/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShaftSubsystem;
import frc.robot.subsystems.ShaftSubsystem.ShaftState;

public class BarrelDefault extends CommandBase {
  ShaftSubsystem shaftSubsystem;
  /**
   * Creates a new BarrelDefault.
   */
  public BarrelDefault(ShaftSubsystem shaftSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shaftSubsystem = shaftSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (shaftSubsystem.getShaftState()) {
      case ShaftState.shooting : {
        if(bb3.get())
          shaftSubsystem.shotCount++;
          state = ShaftState.shootIndex;
        break;
      }
      case shootIndex : {
        spinUpShaft(.5);
        if(!bb3.get()) {
          spinUpShaft(0);
          state = ShaftState.shootWait;
        }
        break;
      }
      case shootWait : {
        if (ShooterMain.readyToShoot) {
          spinUpShaft(.9);
          state = ShaftState.shooting;
        }
        break;
      }
      case fullStop : {
        spinUpShaft(0);
        if (!bb1.get() && bb3.get()) {
          if (bb2.get()) {
            state = ShaftState.runningClear;
          } else {
            state = ShaftState.indexing;
          }
        }
        break;
      }
      case indexing : {
        spinUpShaft(.75);
        if (bb2.get()) {
          state = ShaftState.runningClear;
        }
        if (!bb3.get() ) {
          state = ShaftState.fullStop;
          spinUpShaft(0);
         

        }
        break;
      }
      case runningClear : {
        spinUpShaft(.75);
        if (!bb2.get() || !bb3.get() ) {
          spinUpShaft(0);
          state = ShaftState.fullStop;
          
        }  
        break;  
      }
      case manual : break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
