
package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Encoder;
/**
 *
 */
public class DriveSystem extends SubsystemBase {

    private static WPI_TalonSRX leftFront = new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_FRONT);
    private static WPI_TalonSRX leftRear  = new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_BACK);
    private static WPI_TalonSRX rightFront  = new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_FRONT);
    private static WPI_TalonSRX rightRear  = new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_BACK);

    public static Encoder leftencoder;
    public static Encoder rightencoder;
    public static AHRS gyro;

    public static DifferentialDriveOdometry odometry;

    protected static RamseteController ramseteController = new RamseteController();
    protected static DifferentialDriveKinematics kinematic = new DifferentialDriveKinematics(0.6);
    protected static Field2d field = new Field2d();

    private static double kP = 9.4876;
    private static double kI = 0;
    private static double kD = 0;

    protected static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);

    protected static PIDController leftPID = new PIDController(kP, kI, kD);
    protected static PIDController rightPID = new PIDController(kP, kI, kD);


    private static MotorControllerGroup mLeft = new MotorControllerGroup(leftFront, leftRear);
    private static MotorControllerGroup mRight = new MotorControllerGroup(rightFront, rightRear);
    private static DifferentialDrive mDrive = new DifferentialDrive(mLeft, mRight);

    //public static final DifferentailDriveKinematics KDriveKinematics = new DifferentailDriveKinematics(kTrackWidthMeters);


    private static Encoder rightEncoder = new Encoder(Constants.ENCODER_RIGHT_A, Constants.ENCODER_RIGHT_B);
    private static Encoder leftEncoder = new Encoder(Constants.ENCODER_LEFT_A, Constants.ENCODER_LEFT_B);
    protected static Field2d trajField = new Field2d(); //FOR PATHWEAVER
    private AHRS navx_device = new AHRS(SerialPort.Port.kMXP);


    public DriveSystem() 
    {
        leftRear.setInverted(true);
        leftFront.setInverted(false);
        rightRear.setInverted(true);
        rightFront.setInverted(true);

        rightEncoder.setDistancePerPulse(10./2208.);
        leftEncoder.setDistancePerPulse(10./2208.);
        //2208 pulses per 10ft
         navx_device.enableLogging(true);
        SmartDashboard.putData("trajField", trajField);
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

        mDrive.arcadeDrive(-xSpeed, zRotation);
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

    public double read_distance_right_encoder()
    {
        return rightEncoder.getDistance();
       
    }

    public double read_distance_left_encoder()
    {
        return leftEncoder.getDistance();
    }

    public double read_pulse_right_encoder()
    {
        return rightEncoder.get();
       
    }

    public double read_pulse_left_encoder()
    {
        return leftEncoder.get();
    }

    public double read_velocity_encoder() {
        
        return (rightEncoder.getRate()+leftEncoder.getRate())/2;
        
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
    
    public double getPitch() {
        return navx_device.getPitch();
    }

    public double getRoll() {
        return navx_device.getRoll();
    }

    public double getCompassHeading() {
        return navx_device.getCompassHeading();
    }

    public double getFusedHeading() {
        return navx_device.getFusedHeading();
    }

    public double getLinearWorldAccelX() {
        return navx_device.getWorldLinearAccelX();
    }

    public double getLinearWorldAccelY() {
        return navx_device.getWorldLinearAccelY();
    }

    public double getLinearWorldAccelZ() {
        return navx_device.getWorldLinearAccelZ();
    }

    public static void runTraj(Trajectory trajectory, double timeInSec) {
        Trajectory.State goal = trajectory.sample(timeInSec);
        trajField.setRobotPose(goal.poseMeters);

        var currentPose = odometry.getPoseMeters();

        var chaspeed = ramseteController.calculate(currentPose, goal);

        var wheelSpeeds = kinematic.toWheelSpeeds(chaspeed); // left right speed
        double left = wheelSpeeds.leftMetersPerSecond; // catch sppe from wheelSpeed(with ctrl+left mice)
        double right = wheelSpeeds.rightMetersPerSecond;

        double leftVolt = leftPID.calculate(leftencoder.getRate(), left) + feedforward.calculate(left);
        double rightVolt = rightPID.calculate(rightencoder.getRate(), right) + feedforward.calculate(right);

        mLeft.setVoltage(leftVolt);
        mRight.setVoltage(rightVolt);
        mDrive.feed();

        SmartDashboard.putNumber("leftVolt", leftVolt);
        SmartDashboard.putNumber("rightVolt", rightVolt);

        SmartDashboard.putNumber("left", left);
        SmartDashboard.putNumber("right", right);
        SmartDashboard.putNumber("left_error", leftPID.getPositionError());
        SmartDashboard.putNumber("right_error", rightPID.getPositionError());
        SmartDashboard.putNumber("velocity", goal.velocityMetersPerSecond);

        SmartDashboard.putNumber("chaspeedX", chaspeed.vxMetersPerSecond);
        SmartDashboard.putNumber("chaspeedY", chaspeed.vyMetersPerSecond);

        SmartDashboard.putNumber("errPosX", currentPose.minus(goal.poseMeters).getX());
        SmartDashboard.putNumber("errPosY", currentPose.minus(goal.poseMeters).getY());

        // m_driveSim.setInputs(leftVolt, rightVolt);//for simulation
    }
    public static void updateODO() {
        var gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());
        odometry.update(gyroAngle, leftencoder.getDistance(), rightencoder.getDistance());
        field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putNumber("x", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("heading", odometry.getPoseMeters().getRotation().getDegrees());

        kP = SmartDashboard.getNumber("kP", kP);
        kI = SmartDashboard.getNumber("kI", kI);
        kD = SmartDashboard.getNumber("kD", kD);

        leftPID.setPID(kP, kI, kD);
        rightPID.setPID(kP, kI, kD);
        mDrive.feed();
    }

    public static void setODOPose(Pose2d pose) {
        odometry.resetPosition(pose, pose.getRotation());
        field.setRobotPose(odometry.getPoseMeters());
    }
    public static void resetEnc(){
        rightEncoder.reset();
        leftEncoder.reset();
    }
    public static void resetGyro(){
        gyro.reset();
    }

    public static void resetPIDS() {
        leftPID.reset();
        rightPID.reset();
    }
}




