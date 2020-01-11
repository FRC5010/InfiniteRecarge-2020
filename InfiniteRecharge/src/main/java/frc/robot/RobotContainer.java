
package frc.robot;

import java.util.List;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
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
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private static DriveTrainMain driveTrain;

  public static Joystick driver;
  public static CANSparkMax lDrive1;
  public static CANSparkMax lDrive2;

  public static CANSparkMax rDrive1;
  public static CANSparkMax rDrive2;

  public static CANEncoder lEncoder;
  public static CANEncoder rEncoder;

  public static Pose robotPose;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    init();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driver = new Joystick(0);

  }
  public static void init(){
    
    //Neos HAVE to be in brushless
    lDrive1 = new CANSparkMax(1, MotorType.kBrushless);
    lDrive2 = new CANSparkMax(2, MotorType.kBrushless);

    rDrive1 = new CANSparkMax(3, MotorType.kBrushless);
    rDrive2 = new CANSparkMax(4, MotorType.kBrushless);
    lDrive1.setInverted(false);
    lDrive2.follow(lDrive1, false);

    rDrive1.setInverted(true);
    rDrive2.follow(rDrive1, false);

    lEncoder = lDrive1.getEncoder();
    rEncoder = rDrive1.getEncoder();

    lEncoder.setPositionConversionFactor(Constants.distancePerPulse);
    rEncoder.setPositionConversionFactor(Constants.distancePerPulse);

    lEncoder.setVelocityConversionFactor(Constants.distancePerPulse);
    rEncoder.setVelocityConversionFactor(Constants.distancePerPulse);
    robotPose = new Pose();
    

    driveTrain= new DriveTrainMain();

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.ksVolts,
                                  Constants.kvVoltSecondsPerMeter,
                                  Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        10);

// Create config for trajectory
TrajectoryConfig config =
    new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                        Constants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

// An example trajectory to follow.  All units in meters.
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
    config
);

RamseteCommand ramseteCommand = new RamseteCommand(
    exampleTrajectory,
    robotPose::getPose,
    new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
    new SimpleMotorFeedforward(Constants.ksVolts,
                               Constants.kvVoltSecondsPerMeter,
                               Constants.kaVoltSecondsSquaredPerMeter),
    Constants.kDriveKinematics,
   robotPose::getWheelSpeeds,
    new PIDController(Constants.kPDriveVel, 0, 0),
    new PIDController(Constants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    driveTrain::tankDriveVolts,
    driveTrain
   
);

// Run path following command, then stop at the end.
return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
  }
}
