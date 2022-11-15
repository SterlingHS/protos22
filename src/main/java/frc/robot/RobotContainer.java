
package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;

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
    drivesystem.setDefaultCommand(new Drive( drivesystem, driverController::getLeftX, driverController::getRightX) ); 
    

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
    // The selected command will be run in autonomous
    
    return m_chooser.getSelected();
  }

 public void update_smartboard(){
    SmartDashboard.putBoolean("DIO 0", drivesystem.state_DIO0());
    SmartDashboard.putBoolean("DIO 1", drivesystem.state_DIO1());
    SmartDashboard.putBoolean("DIO 2", drivesystem.state_DIO2());
    SmartDashboard.putBoolean("DIO 3", drivesystem.state_DIO3());
    SmartDashboard.putBoolean("DIO 4", drivesystem.state_DIO4());
    SmartDashboard.putBoolean("DIO 5", drivesystem.state_DIO5());
    SmartDashboard.putBoolean("DIO 6", drivesystem.state_DIO6());
    SmartDashboard.putBoolean("DIO 7", drivesystem.state_DIO7());
    SmartDashboard.putBoolean("DIO 8", drivesystem.state_DIO8());
    SmartDashboard.putBoolean("DIO 9", drivesystem.state_DIO9());
}
}
