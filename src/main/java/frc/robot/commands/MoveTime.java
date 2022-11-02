package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;


/**
 *
 */
    public class MoveTime extends CommandBase {

    private final DriveSystem drivesystem;
    private static long starting_time;

    private static double time;
    private static double speed;

 
    public MoveTime(DriveSystem sub, double speed1, double time1) {
        drivesystem = sub;
        time = time1;
        speed = speed1;

        addRequirements(drivesystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        start_timer();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("Speed: " + speed);
        if(get_timer()<time) drivesystem.forward();
        else drivesystem.stop();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivesystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(get_timer()>time) return true;

        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

    private void start_timer(){
        starting_time = System.currentTimeMillis();
    
      }
    
      private double get_timer(){
        double timer = System.currentTimeMillis() - starting_time;
        return timer;
      }
}