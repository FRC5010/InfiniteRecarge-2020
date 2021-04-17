// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimWithVision;
import frc.robot.mechanisms.Drive;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.VisionSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot3RpShoot2 extends SequentialCommandGroup {
  /** Creates a new Shoot3RpShoot2. */
  public Shoot3RpShoot2(DriveTrainMain driveTrain, VisionSystem visionSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AimWithVision(driveTrain, visionSubsystem, 0.0, 0.0)
     ,
      new ParallelRaceGroup(
        Drive.getAutonomousCommand("paths/RP1.wpilib.json", true)
      ),

      new ParallelDeadlineGroup(
        Drive.getAutonomousCommand("paths/RP2.wpilib.json", false)
      ),
      
      new AimWithVision(driveTrain, visionSubsystem, 0.0, 0.0)
    );
  }
}
