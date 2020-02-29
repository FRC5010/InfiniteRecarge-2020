/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.LoadShaftCommand;
import frc.robot.commands.SpinShooter;
import frc.robot.subsystems.ShaftSubsystem;
import frc.robot.subsystems.ShooterMain;
import frc.robot.subsystems.VisionSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootAndMove extends SequentialCommandGroup {
  /**
   * Creates a new ShootAndMove.
   */
  public ShootAndMove(ShaftSubsystem shaftClimber, ShooterMain shooterMain, VisionSystem visionSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new ParallelRaceGroup(new LoadShaftCommand(shaftClimber,3),new SpinShooter(shooterMain, visionSubsystem))
    );
  }
}
