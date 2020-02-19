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

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;
import frc.robot.commands.IntakeAiming;
import frc.robot.commands.RamseteFollower;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.Vision.VisionSystem;

/**
 * Add your docs here.
 */
public class Drive {
    private static DriveTrainMain driveTrain;
    private static VisionSystem intakeCam;
    private static VisionSystem shooterCam;

    public Joystick driver;
    public static CANSparkMax lDrive1;
    public static CANSparkMax lDrive2;
  
    public static CANSparkMax rDrive1;
    public static CANSparkMax rDrive2;
  
    public static CANEncoder lEncoder;
    public static CANEncoder rEncoder;
  
    public static Pose robotPose;
  
    public JoystickButton intakeSteering;

    public Drive(Joystick driver) {
        init(driver); 
        configureButtonBindings();
    }

      /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    intakeSteering = new JoystickButton(driver, 1);
    intakeSteering.whenPressed(new IntakeAiming(driveTrain, intakeCam, driver));
  }

  public void init(Joystick driver) {
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

      lDrive1.setSmartCurrentLimit(38);
      lDrive2.setSmartCurrentLimit(38);

      rDrive1.setSmartCurrentLimit(38);
      rDrive2.setSmartCurrentLimit(38);


      // lEncoder.setPositionConversionFactor(Constants.distancePerPulse);
      // rEncoder.setPositionConversionFactor(-Constants.distancePerPulse);
      
      // lEncoder.setVelocityConversionFactor(Constants.distancePerPulse);
      // rEncoder.setVelocityConversionFactor(-Constants.distancePerPulse);

    }

    robotPose = new Pose(lEncoder, rEncoder);

    driveTrain = new DriveTrainMain(lDrive1, rDrive1, driver);
    intakeCam = new VisionSystem("intake");
    shooterCam = new VisionSystem("shooter", 26, 0, 90);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(DriveConstants.ksVolts,
        DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter), DriveConstants.kDriveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);
    TrajectoryConfig backwardsConfig = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared)
    //set reversed allows robot to go backwards
    .setReversed(true);
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
        config);

    Trajectory anotherExample = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(1, 1, new Rotation2d(0)),
        new Pose2d(2, -1, new Rotation2d(0)),
        new Pose2d(3, 0, new Rotation2d(0)),
        new Pose2d(3.5, 0, new Rotation2d(0))
        ), config);

        Trajectory driveStraight = TrajectoryGenerator.generateTrajectory(
          List.of(
          new Pose2d(0,0,new Rotation2d(0)), 
          new Pose2d(2.5, 0,new Rotation2d(0)), 
          new Pose2d(5,0,new Rotation2d(0))
         
          ),  config);
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
            backwardsConfig);
            Trajectory revTrajectory = comeBack;
    RamseteCommand ramseteCommand = new RamseteFollower(driveStraight, robotPose::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, robotPose::getWheelSpeeds, new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        driveTrain::tankDriveVolts, driveTrain

    );

    RamseteCommand backCommand = new RamseteFollower(revTrajectory, robotPose::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, robotPose::getWheelSpeeds, new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        driveTrain::tankDriveVolts, driveTrain

    );
    Command result = ramseteCommand.andThen(()->backCommand.schedule());
    // Run path following command, then stop at the end.
    return result;
  }

}
