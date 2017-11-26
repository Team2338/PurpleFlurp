package org.firstinspires.ftc.lib;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 *
 */

public class RobotMap {

    // Drivetrain Hardware
    public static DcMotor frontLeft;
    public static DcMotor frontRight;
    public static DcMotor rearLeft;
    public static DcMotor rearRight;

    // Drivetrain Sensors (Not used)
    public static BNO055IMU imu;

    // Lift Hardware
    public static DcMotorEx lift;
    public static Servo leftServo;
    public static Servo rightServo;

    // Arm Hardware
    public static Servo arm;

    // Arm Sensors
    public static ColorSensor colorSensor; //farts

    public static void init(HardwareMap map) {
        // Drivetrain HARDware
        frontLeft = map.get(DcMotor.class, "frontLeft");
        frontRight = map.get(DcMotor.class, "frontRight");
        rearLeft = map.get(DcMotor.class, "rearLeft");
        rearRight = map.get(DcMotor.class, "rearRight");

        // Drivetrain Sensors
        imu = map.get(BNO055IMU.class, "imu");

        // Lift HARDware
        lift = (DcMotorEx) map.get(DcMotor.class, "lift");
        leftServo = map.get(Servo.class, "leftServo");
        rightServo = map.get(Servo.class, "rightServo");

        // Arm HARDware
        arm = map.get(Servo.class, "arm");

        // Arm Sensors
        colorSensor = map.get(ColorSensor.class, "colorSensor");
    }

}
