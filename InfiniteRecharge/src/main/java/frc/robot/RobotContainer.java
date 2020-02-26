
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.mechanisms.Drive;
import frc.robot.mechanisms.IntakeMech;
import frc.robot.mechanisms.ShaftMechanism;
import frc.robot.mechanisms.Shoot;
import frc.robot.mechanisms.SpinControl;
import frc.robot.mechanisms.TelescopClimb;
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
    shooterVision = new VisionSystem("shooter", 26, 0, 90);
    intakeVision = new VisionSystem("intake", 20, 0, 3.5);
    spinControl = new SpinControl(driver, operator);
    shooter = new Shoot(operator, shooterVision);
    intake = new IntakeMech(operator);
    shaftMechanism = new ShaftMechanism(driver, operator, intake.intakeMain, shooter.shooterMain, shooterVision);
    driveMechanism = new Drive(driver, shooterVision, intakeVision, intake.intakeMain);

    // climb = new TelescopClimb(driver, operator);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return driveMechanism.getAutonomousCommand();
  }
}
