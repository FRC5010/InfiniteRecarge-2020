/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.LoadShaftCommand;
import frc.robot.commands.LowerIntake;
import frc.robot.commands.LowerShaft;
import frc.robot.commands.SpinShooter;
import frc.robot.mechanisms.DriveConstants;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Pose;
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
  VisionSystem visionSubsystem, Pose pose) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    super(
      new ParallelRaceGroup(
        new LoadShaftCommand(shaftClimber, 3, shooterMain, 7), 
        new SpinShooter(shooterMain, visionSubsystem)),
      new ParallelCommandGroup(
        new LowerIntake(intake), 
        new LowerShaft(shaftClimber)),
      new ParallelRaceGroup(
        new RamseteCommand(DriveConstants.moveToTrench, pose::getPose,
          new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
              new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
              DriveConstants.kaVoltSecondsSquaredPerMeter),
              DriveConstants.kDriveKinematics, pose::getWheelSpeeds,
              new PIDController(DriveConstants.kPDriveVel, 0, 0),
              new PIDController(DriveConstants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
              driveTrain::tankDriveVolts, driveTrain),
    
        new IntakeBalls(intake,.8)
      )
    );
  }
} 
