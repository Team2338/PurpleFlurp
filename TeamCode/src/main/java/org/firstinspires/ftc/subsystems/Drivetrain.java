package org.firstinspires.ftc.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.lib.PIDController;
import org.firstinspires.ftc.lib.RobotMap;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 *
 */

public class Drivetrain {

    private static Drivetrain instance = null;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

    private DcMotor frontLeft = RobotMap.frontLeft;
    private DcMotor frontRight = RobotMap.frontRight;
    private DcMotor rearLeft = RobotMap.rearLeft;
    private DcMotor rearRight = RobotMap.rearRight;

    private BNO055IMU imu = RobotMap.imu;

	private final double driveP = 0.01;
	private final double driveI = 0;
	private final double driveD = 0;

	private PIDController drivePID = new PIDController(driveP, driveI, driveD);

	private final double rotateP = 0.001;
	private final double rotateI = 0;
	private final double rotateD = 0;

	private PIDController rotatePID = new PIDController(rotateP, rotateI, rotateD);

    private double maxSpeed = 0.9;

    private Drivetrain() {
       // resetEncoders();

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

		imu.initialize(parameters);
    }

    public void updateDrive() {
        double y = drivePID.getOutput(getPosition());
    	mecanumDrive(0, y, 0);
	}

	public void updateRotate() {
		double rotate = rotatePID.getOutput(getAngle());
		mecanumDrive(0, 0, rotate);
	}

	public void setSetpoint(double setpoint) {
    	drivePID.setSetpoint(setpoint);
    }

    public void rotate(double degrees) {
        rotatePID.setSetpoint(degrees);
    }

    public double getPosition() {
        return (rearLeft.getCurrentPosition() - rearRight.getCurrentPosition()) / 2;
	}

	public double getPosition2() {
        return frontRight.getCurrentPosition();
    }

    public double getPosition3() {
        return rearLeft.getCurrentPosition();
    }

    public double getPosition4() {
        return rearRight.getCurrentPosition();
    }


	public boolean inDriveTolerance() {
    	return drivePID.inTolerance(5);
    }

    public boolean inRotateTolerance() {
    	return rotatePID.inTolerance(1);
	}

    public double getAngle() {
    	return imu.getAngularOrientation().firstAngle;
	}

    public void mecanumDrive(double x, double y, double rotation) {
        frontLeft.setPower(normalize(x + y + rotation) * maxSpeed);
        frontRight.setPower(normalize(x - y + rotation) * maxSpeed);
        rearLeft.setPower(normalize(-x + y + rotation) * maxSpeed);
        rearRight.setPower(normalize(-x - y + rotation) * maxSpeed);
    }

    //LEFT IS POSITIVE, RIGHT IS NEGATIVE TO GO FORWARD
    public void BackwardKnock () {       //Makes the robot go backward at low power
        frontLeft.setPower(-0.4);
        frontRight.setPower(0.4);
        rearLeft.setPower(-0.4);
        rearRight.setPower(0.4);
    }

    public void StoptheMotor () {       //Stops the robot
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    public void ForwardKnock () {      //Goes forward at low power
        frontRight.setPower(-0.4);
        frontLeft.setPower(0.4);
        rearRight.setPower(-0.4);
        rearLeft.setPower(0.4);
    }

    public void VerytinyF () {
        frontRight.setPower(-0.2);
        frontLeft.setPower(0.2);
        rearRight.setPower(-0.2);
        rearLeft.setPower(0.2);
    }

    public void VerytinyB () {
        frontRight.setPower(0.1);
        frontLeft.setPower(-0.1);
        rearRight.setPower(0.1);
        rearLeft.setPower(-0.1);
    }

    public void GetIntoBoxF () {     //Forward at a higher power than before
        frontRight.setPower(-0.6);
        frontLeft.setPower(0.6);
        rearRight.setPower(-0.6);
        rearLeft.setPower(0.6);
    }

    public void GetIntoBoxB () { //Backward Version
        frontRight.setPower(0.6);
        frontLeft.setPower(-0.6);
        rearLeft.setPower(-0.6);
        rearRight.setPower(0.6);
    }

    public void RampUp () {  //Extreme Power
        frontLeft.setPower(0.8);
        frontRight.setPower(-0.8);
        rearLeft.setPower(0.8);
        rearRight.setPower(-0.8);
    }

    public void RampBack () { //Opposite of RampUp
        frontLeft.setPower(-0.8);
        frontRight.setPower(0.8);
        rearRight.setPower(0.8);
        rearLeft.setPower(-0.8);
    }

    public void StrafeLeft () { //CHANGE THESE QUICKLY
        frontLeft.setPower(0.8);
        rearRight.setPower(0.8);
        frontRight.setPower(-0.8);
        rearLeft.setPower(-0.8);
    }

    public void StrafeRight () { // I don't even need this so ignore it
        frontLeft.setPower(-0.4);
        rearRight.setPower(-0.4);
        frontRight.setPower(0.4);
        rearLeft.setPower(0.4);
    }

    public void TurnRight () { //TURNS RIGHT WHEELS BACKWARDS
        frontRight.setPower(0.5);
        rearRight.setPower(0.5);
    }

    public void TurnLeft () { //TURNS LEFT FORWARDS
        frontLeft.setPower(0.5);
        rearLeft.setPower(0.5);
    }

    public void TurnLeftB () { //Turns LEFT BACKWARDS
        rearRight.setPower(-0.5);
        rearLeft.setPower(-0.5);
    }

    public void TurnRightF () { //Turns RIGHT FORWARDS
        rearRight.setPower(-0.5);
        frontRight.setPower(-0.5);
    }

    public void resetEncoders() {
		frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	private double normalize(double number) {
		if (number > 1) {
			return 1;
		} else if (number < -1) {
			return -1;
		} else {
			return number;
		}
	}
}
