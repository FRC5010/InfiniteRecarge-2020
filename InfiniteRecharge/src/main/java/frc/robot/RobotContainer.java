
package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.RamseteFollower;
import frc.robot.commands.auto.ShootAndMove;
import frc.robot.mechanisms.Drive;
import frc.robot.mechanisms.DriveConstants;
import frc.robot.mechanisms.IntakeMech;
import frc.robot.mechanisms.ShaftMechanism;
import frc.robot.mechanisms.Shoot;
import frc.robot.mechanisms.SpinControl;
import frc.robot.mechanisms.TelescopClimb;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.VisionSystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's Mechanisms are defined here...
  private Drive driveMechanism;
  private SpinControl spinControl;
  private ShaftMechanism shaftMechanism;
  private Joystick driver;
  private Joystick operator;
  public Shoot shooter;
  private IntakeMech intake;
  private TelescopClimb climb;
  private VisionSystem shooterVision;
  private VisionSystem intakeVision;
  private Pose robotPose;
  private DriveTrainMain driveTrain;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // motors 1-4 are drivetrain,
    // motor 5 is the shooter,
    // motor 6 and 7 are the climb,
    // motor 8 is the shaft,
    // motor 9 is the intake
    // motor 10 is the spinner
    driver = new Joystick(0);
    operator = new Joystick(1);
    Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay);
    shooterVision = new VisionSystem("shooter", 26, 0, 90, ControlConstants.shooterVisionColumn);
    //shooterVision.getRawValues().calibarateCamAngle();
    intakeVision = new VisionSystem("intake", 20, 0, 3.5, ControlConstants.intakeVisionColumn);
    
    shooter = new Shoot(operator, driver, shooterVision);
    intake = new IntakeMech(operator);
    shaftMechanism = new ShaftMechanism(driver, operator, intake.intakeMain, shooter.shooterMain, shooterVision);
    //driveMechanism = new Drive(driver, shooterVision, intakeVision, intake.intakeMain);
    spinControl = new SpinControl(driver, operator, shaftMechanism.getSubsystem());
    climb = new TelescopClimb(driver, operator);


    robotPose = Drive.robotPose;
    //driveTrain = driveMechanism.driveTrain;

    //buttons driver
    //x = spin spinner(rotation controller)
    //joysticks move robot
    //left bumper = toggle barrel height
    //b = angle towards target
    //right bumper = toggle spinner height
    //y = spin spinner(position controller)
    //


    //buttons co-driver(operator cause jackson)
    //y = spin shaft/conveyer belt
    //a = spins shooter to the correct speed
    //right trigger = spin intake in
    //left trigger = spin intake out
    //right bumper = toggle intake height

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
  //  // Create a voltage constraint to ensure we don't accelerate too fast
   

  //   // Create config for trajectory
    
  //   // An example trajectory to follow. All units in meters.
  //   Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  //       // Start at the origin facing the +X direction
  //       new Pose2d(0, 0, new Rotation2d(0)),
  //       // Pass through these two interior waypoints, making an 's' curve path
  //       List.of(
  //         new Translation2d(1, 1),
  //         new Translation2d(2, -1)
  //       ),
  //       // End 3 meters straight ahead of where we started, facing forward
  //       new Pose2d(3, 0, new Rotation2d(0)),
  //       // Pass config
  //       DriveConstants.config);

  //   Trajectory anotherExample = TrajectoryGenerator.generateTrajectory(
  //     List.of(
  //       new Pose2d(0, 0, new Rotation2d(0)),
  //       new Pose2d(1, 1, new Rotation2d(0)),
  //       new Pose2d(2, -1, new Rotation2d(0)),
  //       new Pose2d(3, 0, new Rotation2d(0)),
  //       new Pose2d(3.5, 0, new Rotation2d(0))
  //       ), DriveConstants.config);

  //       Trajectory driveStraight = TrajectoryGenerator.generateTrajectory(
  //         List.of(
  //         new Pose2d(0,0,new Rotation2d(0)), 
  //         new Pose2d(2.5, 0,new Rotation2d(0)), 
  //         new Pose2d(5,0,new Rotation2d(0))
         
  //         ),  DriveConstants.config);
  //         Pose2d endPoint = new Pose2d(9, 0, new Rotation2d(0));
  //         Trajectory comeBack = TrajectoryGenerator.generateTrajectory(
  //           // Start at the origin facing the +X direction
  //          List.of(
  //            new Pose2d(5,0,new Rotation2d(0)), 
  //          new Pose2d(2.5, 0,new Rotation2d(0)), 
  //           new Pose2d(0,0,new Rotation2d(0))
  //          )
  //          ,
  //           // Pass config
  //           DriveConstants.backwardsConfig);
  //           Trajectory revTrajectory = comeBack;
  //   RamseteCommand ramseteCommand = new RamseteFollower(driveStraight, robotPose::getPose,
  //       new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
  //       new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
  //           DriveConstants.kaVoltSecondsSquaredPerMeter),
  //       DriveConstants.kDriveKinematics, robotPose::getWheelSpeeds, new PIDController(DriveConstants.kPDriveVel, 0, 0),
  //       new PIDController(DriveConstants.kPDriveVel, 0, 0),
  //       // RamseteCommand passes volts to the callback
  //       driveTrain::tankDriveVolts, driveTrain 

  //   );

  //   RamseteCommand backCommand = new RamseteFollower(revTrajectory, robotPose::getPose,
  //       new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
  //       new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
  //           DriveConstants.kaVoltSecondsSquaredPerMeter),
  //       DriveConstants.kDriveKinematics, robotPose::getWheelSpeeds, new PIDController(DriveConstants.kPDriveVel, 0, 0),
  //       new PIDController(DriveConstants.kPDriveVel, 0, 0),
  //       // RamseteCommand passes volts to the callback
  //       driveTrain::tankDriveVolts, driveTrain

  //   );
  //   Command result = ramseteCommand.andThen(()->backCommand.schedule());
    
    // Run path following command, then stop at the end.
    return new ShootAndMove(shaftMechanism.shaftClimber, shooter.shooterMain, driveTrain, shooterVision, robotPose);
  }
}
