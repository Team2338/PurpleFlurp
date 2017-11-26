package org.firstinspires.ftc.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.lib.RobotMap;

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

    private double maxSpeed = 0.9;

    private Drivetrain() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void mecanumDrive(double x, double y, double rotation) {
        frontLeft.setPower(normalize(x + y + rotation) * maxSpeed);
        frontRight.setPower(normalize(x - y + rotation) * maxSpeed);
        rearLeft.setPower(normalize(-x + y + rotation) * maxSpeed);
        rearRight.setPower(normalize(-x - y + rotation) * maxSpeed);
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

    public void ForwardKnock () {       //Makes the robot go forward at low power
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

    public void BackwardKnock () {      //Goes backward at low power
        frontRight.setPower(-0.4);
        frontLeft.setPower(0.4);
        rearRight.setPower(-0.4);
        rearLeft.setPower(0.4);
    }

    public void GetIntoBox () {     //Forward at a higher power than before
        frontRight.setPower(0.6);
        frontLeft.setPower(-0.6);
        rearRight.setPower(0.6);
        rearLeft.setPower(-0.6);
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
}
