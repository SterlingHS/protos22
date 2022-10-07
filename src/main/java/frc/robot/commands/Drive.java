
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveSystem;
import java.util.function.DoubleSupplier;

   

/**
 *
 */
public class Drive extends CommandBase {

    private final DriveSystem drivesystem;
    private final DoubleSupplier forward;
    private final DoubleSupplier lateral;
    private final DoubleSupplier rotation;

    public Drive(DriveSystem subsystem, DoubleSupplier forward_im, DoubleSupplier lateral_im, DoubleSupplier rotation_im) 
    {
        drivesystem = subsystem;
        forward = forward_im;
        lateral = lateral_im;
        rotation = rotation_im;
        addRequirements(drivesystem);
    }
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            drivesystem.mecanumDrive(forward.getAsDouble(), lateral.getAsDouble(), rotation.getAsDouble(), RobotMap.DRIVER_SLOWDOWN);
            // System.out.println("Stopped...");
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
           drivesystem.stop();
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public boolean runsWhenDisabled() {
            return false;
        }
    
}
