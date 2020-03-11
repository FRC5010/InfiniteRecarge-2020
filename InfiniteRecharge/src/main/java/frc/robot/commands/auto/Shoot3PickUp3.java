/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimWithVision;
import frc.robot.commands.BarrelDefault;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.LoadShaftCommand;
import frc.robot.commands.LowerIntake;
import frc.robot.commands.LowerShaft;
import frc.robot.commands.RaiseBarrel;
import frc.robot.commands.RamseteFollower;
import frc.robot.commands.ShooterDefault;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.TurnToAngle;
import frc.robot.mechanisms.Drive;
import frc.robot.mechanisms.DriveConstants;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShaftSubsystem;
import frc.robot.subsystems.ShooterMain;
import frc.robot.subsystems.VisionSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Shoot3PickUp3 extends SequentialCommandGroup {
  /**
   * Creates a new Shoot3PickUp3.
   */
  public Shoot3PickUp3(ShaftSubsystem shaftClimber, ShooterMain shooterMain,IntakeSubsystem intake, DriveTrainMain driveTrain,
  VisionSystem visionSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    super(
      new RaiseBarrel(shaftClimber),
      new AimWithVision(driveTrain, visionSubsystem, 0.0, 0.0)
     ,
      new ParallelRaceGroup(
        new LoadShaftCommand(shaftClimber, 3, shooterMain, 7), 
        new SpinShooter(shooterMain, visionSubsystem)
      ),

      new ParallelCommandGroup(
        new LowerIntake(intake), 
        new LowerShaft(shaftClimber)
      ),

      new ParallelRaceGroup(
        new RamseteFollower(DriveConstants.moveToTrenchPickUp3, false),
        new BarrelDefault(shaftClimber),
        new IntakeBalls(intake,.9)
      ),

      new ParallelDeadlineGroup(
        new RamseteFollower(DriveConstants.moveForward,false),
        new RaiseBarrel(shaftClimber)
      ),
      
      new ParallelRaceGroup(
            new AimWithVision(driveTrain, visionSubsystem, 0.0, 0.0)
           // new SpinShooter(shooterMain, visionSubsystem)
        ),

      new ParallelDeadlineGroup(
          new LoadShaftCommand(shaftClimber,3,shooterMain,10), 
          new SpinShooter(shooterMain, visionSubsystem), 
          new IntakeBalls(intake, .5) 
      )
    );
  }
} 
