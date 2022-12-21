
package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;


/*Left off on loop() function, start() function. After that, trajectory implementation must be done by
setting up trajs 
*/



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
  static SendableChooser<String> m_chooser = new SendableChooser<>();

  static Timer timer = new Timer();

  protected static String autoSelected;

  

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    int trajectoryAmount = 0; //Put number of trajectories here

    String[] trajectoryJSON = {
      //Put JSON names here, or find alternative
    };

    Trajectory[] trajectory = new Trajectory[trajectoryAmount];


    chooserSetting();

    for (int i = 0; i < trajectoryAmount; i++) {
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON[i]);
        trajectory[i] = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON[i] + "/n" + ex.getMessage(), ex.getStackTrace());
      }
    }

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

  public static void start() {
    DriveSystem.resetEnc();
    DriveSystem.resetGyro();
    DriveSystem.resetPIDS();
    autoSelected = m_chooser.getSelected();


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */

  public void getAutonomousCommand() {
    // The selected command will be run in autonomous
    
    m_chooser.getSelected();
  }

  public static void loop() {
    DriveSystem.updateODO();
    SmartDashboard.putNumber("Time", timer.get());
    switch (autoSelected)
  }

  public static void chooserSetting() {
    //m_chooser.setDefualtOption("Do Nothing", kDoNothing);
    //Set up traj choices here
  
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
