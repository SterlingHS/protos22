
package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;

//Add autoconstants and driveconstants

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSystem drivesystem = new DriveSystem();

  // Joysticks
  private final XboxController driverController = new XboxController(RobotMap.JOYDRIVER_USB_PORT);
  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    drivesystem.setDefaultCommand(new Drive( drivesystem, driverController::getLeftY, driverController::getLeftX) ); 
    

    m_chooser.setDefaultOption("string",new MoveTime(drivesystem, 0.5,1000));
  }

  /*public static RobotContainer getInstance() {
    return m_robotContainer;
  }*/

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}
  

  public XboxController getDriverController() {
    return driverController;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */

  public Command getAutonomousCommand() {

    //Voltage contstraints uses information from DriveConstants to ensure acceptable acceleration
    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(//Note: Get Drive Constants File
            Constants.ksVolts,
            Constants.ksVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
          Constants.kDriveKinematics, 10);
      
    TrajectoryConfig config =
      new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
      config.setKinematics(Constants.kDriveKinematics);
      config.addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory = PathweaverSubsystem.trajectory;

    RamseteCommand ramseteCommand =
      new RamseteCommand(
        exampleTrajectory,
        drivesystem::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(
          Constants.ksVolts,
          Constants.ksVoltSecondsPerMeter,
          Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        DriveSystem::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        drivesystem::tankDriveVolts,
        drivesystem);

  

    
    DriveSystem.setODOPose(exampleTrajectory.getInitialPose());
    DriveSystem.resetEnc();

    return ramseteCommand.andThen(() -> drivesystem.tankDriveVolts(0,0));
    //return m_chooser.getSelected();
  }

 public void update_smartboard(){
        // SmartDashboard.putBoolean("DIO 9", drivesystem.state_DIO9());
        SmartDashboard.putNumber("Right Pulse", drivesystem.read_pulse_right_encoder());
        SmartDashboard.putNumber("Left Pulse", drivesystem.read_pulse_left_encoder());
        SmartDashboard.putNumber("Right Distance", drivesystem.read_distance_right_encoder());
        SmartDashboard.putNumber("Left Distance", drivesystem.read_distance_left_encoder());
        SmartDashboard.putNumber("Velocity", drivesystem.read_velocity_encoder());
        SmartDashboard.putNumber("Angle", drivesystem.getAngle360());
        SmartDashboard.putNumber("Pitch", drivesystem.getPitch());
        SmartDashboard.putNumber("Roll", drivesystem.getRoll());
        SmartDashboard.putNumber("Compass Heading", drivesystem.getCompassHeading());
        SmartDashboard.putNumber("Fused Heading", drivesystem.getFusedHeading());
        SmartDashboard.putNumber("Linear World Accel X", drivesystem.getLinearWorldAccelX());
        SmartDashboard.putNumber("Linear World Accel Y", drivesystem.getLinearWorldAccelY());
        SmartDashboard.putNumber("Linear World Accel Z", drivesystem.getLinearWorldAccelZ());

}
}
