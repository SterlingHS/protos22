
package frc.robot.subsystems;


import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.SerialPort;


/**
 *
 */
public class DriveSystem extends SubsystemBase {

private WPI_TalonSRX leftFront = new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_FRONT);
private WPI_TalonSRX leftRear  = new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_BACK);
private WPI_TalonSRX rightFront  = new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_FRONT);
private WPI_TalonSRX rightRear  = new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_BACK);

private MotorControllerGroup mLeft = new MotorControllerGroup(leftFront, leftRear);
private MotorControllerGroup mRight = new MotorControllerGroup(rightFront, rightRear);
private DifferentialDrive mDrive = new DifferentialDrive(mLeft, mRight);

private AHRS navxDevice;

    /**
    *
    */
    public DriveSystem() {
 
    leftRear.setInverted(true);
    leftFront.setInverted(false);
    rightRear.setInverted(true);
    rightFront.setInverted(true);



    navxDevice = new AHRS(SerialPort.Port.kMXP);  
    navxDevice.enableLogging(true);

}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }


    public void ArcDrive(double xSpeed, double zRotation, double slowdownFactor) 
    {
        if(slowdownFactor < 1 && slowdownFactor >= 0)
        {
            xSpeed*=slowdownFactor;
            zRotation*=slowdownFactor;
        }

        mDrive.arcadeDrive(xSpeed, zRotation);
    }

    public void stop() {
        leftFront.stopMotor();
        rightFront.stopMotor();
        leftRear.stopMotor();
        rightRear.stopMotor();
    }
    public void turnRight() {
        ArcDrive(0,0.5,1);
    }

    public void turnLeft(){
        ArcDrive(0,-0.5,1);
    }
    public void forward(){
        ArcDrive(.5,0,1);
    }
    public void forwardSpeed(double xSpeed){
        ArcDrive(xSpeed,0,1);
    }
    public void backward(){
        ArcDrive(-0.5,0,1);
    }

    public void calibrateGyro()
    {
        navxDevice.calibrate();
    }

    public boolean iscalibrating()
    {
        return navxDevice.isCalibrating();
    }

    public void resetAngle()
    {
        navxDevice.reset();
    }

    public double getAngle()
    {
        return navxDevice.getAngle();
    }

    public double getAngle360()
    {
        double angle = navxDevice.getAngle();

        double correctedAngle = angle % 360;
        if (correctedAngle < 0){
            correctedAngle += 360;
        }
        return correctedAngle;
    }


}

