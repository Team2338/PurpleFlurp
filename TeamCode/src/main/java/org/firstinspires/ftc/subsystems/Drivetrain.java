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

	private final double driveP = 0.001;
	private final double driveI = 0;
	private final double driveD = 0;

	private PIDController drivePID = new PIDController(driveP, driveI, driveD);

	private final double rotateP = 0.001;
	private final double rotateI = 0;
	private final double rotateD = 0;

	private PIDController rotatePID = new PIDController(rotateP, rotateI, rotateD);

    private double maxSpeed = 0.9;

    private Drivetrain() {
        resetEncoders();

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

		imu.initialize(parameters);
    }

    public void updateDrive() {
    	double y = drivePID.getOutput(frontLeft.getCurrentPosition());
    	mecanumDrive(0, y, 0);
	}

	public void updateRotate() {
		double rotate = rotatePID.getOutput(getAngle());
		mecanumDrive(0, 0, rotate);
	}

	public void setSetpoint(double setpoint) {
    	resetEncoders();
    	drivePID.setSetpoint(setpoint);
    }

    public double getPosition() {
    	return frontLeft.getCurrentPosition();
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

    public void GetIntoBox () {     //Forward at a higher power than before
        frontRight.setPower(-0.6);
        frontLeft.setPower(0.6);
        rearRight.setPower(-0.6);
        rearLeft.setPower(0.6);
    }

    public void RampUp () {
        frontLeft.setPower(0.8);
        frontRight.setPower(-0.8);
        rearLeft.setPower(0.8);
        rearRight.setPower(-0.8);
    }

    //NOTE: THESE ARE ALL FOR FUTURE USE BELOW

    //This moves the robot a little bit to the left before turning
    public void giveClearance () {

    }

    //This is supposed to turn the robot 90 degrees so it can place the glyph in the box
    public void Turn () {

    }

    //this is to move the robot forward towards the glyph
    public void moveForward () {

    }

    //After releasing the claw by using LIFT, make robot move back near center
    public void moveBack () {

    }

    //Turn around the robot to set up for Driver Controlled Period
    public void turnAround () {

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
