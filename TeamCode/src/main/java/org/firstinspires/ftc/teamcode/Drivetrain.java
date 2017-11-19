package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

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
}
