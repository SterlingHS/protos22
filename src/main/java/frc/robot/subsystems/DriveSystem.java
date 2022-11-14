
package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
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

    //public static final DifferentailDriveKinematics KDriveKinematics = new DifferentailDriveKinematics(kTrackWidthMeters);

    DigitalInput input0 = new DigitalInput(0);
    DigitalInput input1 = new DigitalInput(1);
    DigitalInput input2 = new DigitalInput(2);
    DigitalInput input3 = new DigitalInput(3);
    DigitalInput input4 = new DigitalInput(4);
    DigitalInput input5 = new DigitalInput(5);
    DigitalInput input6 = new DigitalInput(6);
    DigitalInput input7 = new DigitalInput(7);
    DigitalInput input8 = new DigitalInput(8);
    DigitalInput input9 = new DigitalInput(9);

    Encoder rightEncoder, leftEncoder;
    AHRS navx_device;

    public DriveSystem() 
    {
        leftRear.setInverted(true);
        leftFront.setInverted(false);
        rightRear.setInverted(true);
        rightFront.setInverted(true);

        rightEncoder = new Encoder(Constants.ENCODER_RIGHT_A, Constants.ENCODER_RIGHT_B);
        leftEncoder = new Encoder(Constants.ENCODER_LEFT_A, Constants.ENCODER_LEFT_B);

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


    public void arcDrive(double xSpeed, double zRotation, double slowdown_factor) 
    {
        if(slowdown_factor < 1 && slowdown_factor >= 0)
        {
            xSpeed*=slowdown_factor;
            zRotation*=slowdown_factor;
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
        arcDrive(0,0.5,1);
    }

    public void turnLeft(){
        arcDrive(0,-0.5,1);
    }
    public void forward(){
        arcDrive(0.5,0,1);
    }
    public void forwardSpeed(double xSpeed){
        arcDrive(xSpeed,0,1);
    }
    public void backward(){
        arcDrive(-0.5,0,1);
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

    public boolean DIO_STATE0(){
        return input0.get();
    }
    public boolean DIO_STATE1(){
        return input1.get();
    }
    public boolean DIO_STATE2(){
        return input2.get();
    }
    public boolean DIO_STATE3(){
        return input3.get();
    }
    public boolean DIO_STATE4(){
        return input4.get();
    }
    public boolean DIO_STATE5(){
        return input5.get();
    }
    public boolean DIO_STATE6(){
        return input6.get();
    }
    public boolean DIO_STATE7(){
        return input7.get();
    }
    public boolean DIO_STATE8(){
        return input8.get();
    }
    public boolean DIO_STATE9(){
        return input9.get();
    }
}




