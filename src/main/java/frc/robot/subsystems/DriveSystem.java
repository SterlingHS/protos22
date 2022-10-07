
package frc.robot.subsystems;


import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.SerialPort;


/**
 *
 */
public class DriveSystem extends SubsystemBase {

private WPI_TalonSRX leftFront = new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_FRONT);
private WPI_TalonSRX leftRear  = new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_BACK);
private WPI_TalonSRX rightFront  = new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_FRONT);
private WPI_TalonSRX rightRear  = new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_BACK);

private MotorControllerGroup m_left = new MotorControllerGroup(leftFront, leftRear);
private MotorControllerGroup m_right = new MotorControllerGroup(rightFront, rightRear);
private DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

private AHRS navx_device;

    /**
    *
    */
    public DriveSystem() {
 
    leftRear.setInverted(true);
    leftFront.setInverted(false);
    rightRear.setInverted(true);
    rightFront.setInverted(true);



    navx_device = new AHRS(SerialPort.Port.kMXP);  
    navx_device.enableLogging(true);

}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }


    public void mecanumDrive(double X, double Y, double Z, double slowdown_factor) 
    {
        /*if(slowdown_factor < 1 && slowdown_factor >= 0)
        {
            X*=slowdown_factor;
            Y*=slowdown_factor;
            Z*=slowdown_factor;
        }*/

        mecanumDrive1.driveCartesian(-X, Y, -Z);
    }

    public void stop() {
        leftFront.stopMotor();
        rightFront.stopMotor();
        leftRear.stopMotor();
        rightRear.stopMotor();
    }
    public void turnRight() {
        mecanumDrive(0,0,0.5,1);
    }

    public void turnLeft(){
        mecanumDrive(0,0,-0.5,1);
    }
    public void forward(){
        mecanumDrive(0,-0.35,0,1);
    }
    public void forwardSpeed(double speed){
        mecanumDrive(0,-speed,0,1);
    }
    public void backward(){
        mecanumDrive(0,0.35,0,1);
    }

    public void calibrateGyro()
    {
        navx_device.calibrate();
    }

    public boolean iscalibrating()
    {
        return navx_device.isCalibrating();
    }

    public void resetAngle()
    {
        navx_device.reset();
    }

    public double getAngle()
    {
        return navx_device.getAngle();
    }

    public double getAngle360()
    {
        double angle = navx_device.getAngle();

        double correctedAngle = angle % 360;
        if (correctedAngle < 0){
            correctedAngle += 360;
        }
        return correctedAngle;
    }


}

