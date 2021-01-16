/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.mechanisms;

import java.util.List;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.ControlConstants;
import frc.robot.Robot;
import frc.robot.commands.AimWithVision;
import frc.robot.commands.RamseteFollower;
import frc.robot.commands.SeekTarget;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.auto.GalacticSearch;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.ShaftSubsystem;
import frc.robot.subsystems.VisionSystem;

/**
 * Add your docs here.
 */
public class Drive {
  public static DriveTrainMain driveTrain;
  private static VisionSystem intakeCam;
  private static VisionSystem shooterCam;
  private static IntakeSubsystem intakeSystem;
  private static ShaftSubsystem shaftSubsystem;

  public Joystick driver;
  public static CANSparkMax lDrive1;
  public static CANSparkMax lDrive2;

  public static CANSparkMax rDrive1;
  public static CANSparkMax rDrive2;

  public static CANEncoder lEncoder;
  public static CANEncoder rEncoder;

  public static Pose robotPose;

  public JoystickButton intakeAimButton;
  public JoystickButton shooterAimButton;
  public POVButton turnToAngleButton;

  public JoystickButton intakeDriveButton;
  public JoystickButton autoNavButton;

  public Drive(Joystick driver, VisionSystem shooterVision, VisionSystem intakeVision, IntakeSubsystem intakeSystem, ShaftSubsystem shaftSubsystem) {
    init(driver, shooterVision, intakeVision, intakeSystem, shaftSubsystem);
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    intakeAimButton = new JoystickButton(driver, ControlConstants.intakeAimButton);
    intakeAimButton.whileHeld(new AimWithVision(driveTrain, intakeCam, driver, 0));
    shooterAimButton = new JoystickButton(driver, ControlConstants.shooterAimButton);
    shooterAimButton.whileHeld(new AimWithVision(driveTrain, shooterCam, driver, 0));
    turnToAngleButton = new POVButton(driver, ControlConstants.turnToAngleButton);
    turnToAngleButton.whenPressed(new TurnToAngle(driveTrain, robotPose, shooterCam.getAngleX()));
    autoNavButton = new JoystickButton(driver,  ControlConstants.autoNavButton);
    autoNavButton.whenPressed(new GalacticSearch(driveTrain, intakeCam, robotPose, intakeSystem, shaftSubsystem));
    // intakeDriveButton = new JoystickButton(drivgber, ControlConstants.startClimb);
    // intakeDriveButton.whenPressed(new ParallelCommandGroup(new AimWithVision(driveTrain, intakeCam, 30, 0.2), new IntakeBalls(intakeSystem, 0.7)));
  }

  public static void setCurrentLimits(int currentLimit) {
    lDrive1.setSmartCurrentLimit(currentLimit);
    lDrive2.setSmartCurrentLimit(currentLimit);
    rDrive1.setSmartCurrentLimit(currentLimit);
    rDrive2.setSmartCurrentLimit(currentLimit);
}



  public void init(Joystick driver, VisionSystem shooterVision, VisionSystem intakeVision, IntakeSubsystem intakeSubsystem, ShaftSubsystem shaftSubsystem) {
    if (RobotBase.isReal()) {
      this.driver = driver;
        // Neos HAVE to be in brushless
      lDrive1 = new CANSparkMax(1, MotorType.kBrushless);
      lDrive2 = new CANSparkMax(2, MotorType.kBrushless);

      rDrive1 = new CANSparkMax(3, MotorType.kBrushless);
      rDrive2 = new CANSparkMax(4, MotorType.kBrushless);
      lDrive1.setInverted(DriveConstants.leftReversed);
      lDrive2.follow(lDrive1, false);

      rDrive1.setInverted(true);
      rDrive2.follow(rDrive1, false);
      rDrive2.setInverted(false);

      lEncoder = lDrive1.getEncoder();
      rEncoder = rDrive1.getEncoder();

      setCurrentLimits(ControlConstants.driveTrainCurrentLimit);

      // lEncoder.setPositionConversionFactor(Constants.distancePerPulse);
      // rEncoder.setPositionConversionFactor(-Constants.distancePerPulse);
      
      // lEncoder.setVelocityConversionFactor(Constants.distancePerPulse);
      // rEncoder.setVelocityConversionFactor(-Constants.distancePerPulse);

    }

    robotPose = new Pose(lEncoder, rEncoder);
    shooterCam = shooterVision;
    driveTrain = new DriveTrainMain(lDrive1, rDrive1, driver, robotPose);
    intakeCam = intakeVision;
    intakeSystem = intakeSubsystem;
    this.shaftSubsystem = shaftSubsystem;

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
   

    // Create config for trajectory
    
    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
          new Translation2d(1, 1),
          new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        DriveConstants.config);

    Trajectory anotherExample = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(1, 1, new Rotation2d(0)),
        new Pose2d(2, -1, new Rotation2d(0)),
        new Pose2d(3, 0, new Rotation2d(0)),
        new Pose2d(3.5, 0, new Rotation2d(0))
        ), DriveConstants.config);

        Trajectory driveStraight = TrajectoryGenerator.generateTrajectory(
          List.of(
          new Pose2d(0,0,new Rotation2d(0)), 
          new Pose2d(2.5, 0,new Rotation2d(0)), 
          new Pose2d(5,0,new Rotation2d(0))
         
          ),  DriveConstants.config);
          Pose2d endPoint = new Pose2d(9, 0, new Rotation2d(0));
          Trajectory comeBack = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
           List.of(
             new Pose2d(5,0,new Rotation2d(0)), 
           new Pose2d(2.5, 0,new Rotation2d(0)), 
            new Pose2d(0,0,new Rotation2d(0))
           )
           ,
            // Pass config
            DriveConstants.backwardsConfig);
            Trajectory revTrajectory = comeBack;
    RamseteCommand ramseteCommand = new RamseteFollower(driveStraight,true);

    RamseteCommand backCommand = new RamseteFollower(revTrajectory,true);
    Command result = ramseteCommand.andThen(()->backCommand.schedule());
    
    // Run path following command, then stop at the end.
    return result;
  }

}
