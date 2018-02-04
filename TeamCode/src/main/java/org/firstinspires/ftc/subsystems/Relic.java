package org.firstinspires.ftc.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.lib.PIDController;
import org.firstinspires.ftc.lib.RobotMap;

/**
 *
 */

public class Relic {

    private static Relic instance = null;

    public static Relic getInstance() {
        if (instance == null) {
            instance = new Relic();
        }
        return instance;
    }

    //ACTIVATE ALL THE ROBOTMAPS WHEN IN USE
    private Servo relicLeft = RobotMap.relicLeft;
    private Servo relicRight = RobotMap.relicRight;
    private DcMotor relicMotor = RobotMap.relicMotor;
    private DcMotor relicExtend = RobotMap.relicExtend;

    private double maxSpeed = 0.7;
    private double flipMaxSpeed = 0.85;

    private int position = 0;

    public void relicSetPoint() {
        if (position == 0) {
            relicStart();
        } else if (position == 1) {
            relicOpen();
        } else if (position == 2) {
            relicClose();
        }
    }

    private Relic() {
        relicMotor.setDirection(DcMotor.Direction.FORWARD);
        relicMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //Both motors
        relicExtend.setDirection(DcMotor.Direction.FORWARD);
        relicExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getPosition() {
        return relicMotor.getCurrentPosition();
    }

    public void relicDrive(double q) {          //Check if this works
        relicMotor.setPower(normalize(q) * flipMaxSpeed);
    }

    public void relicReverse(double z) {
        relicMotor.setPower(reversenormalize(z) * flipMaxSpeed);
    }

    public void relicExtension(double u) {
        relicExtend.setPower(extendnormalize(u) * maxSpeed);
    }

    public void relicOpen() {
        relicLeft.setPosition(0.7);
        relicRight.setPosition(0.3); // this is good
    }

    public void relicClose() {
        relicLeft.setPosition(1);
        relicRight.setPosition(0.2); // this is good
    }

    public void relicStart() {
        relicLeft.setPosition(0);
        relicRight.setPosition(0.9);
    }

    public void raise() { //MAKE THE RELIC LIKE THE LIFT
        position += 1;
        position = Range.clip(position, 0, 2);
        relicSetPoint();
    }

    public void lower() {
        position -= 1;
        position = Range.clip(position, 0, 2);
        relicSetPoint();
    }

    public void activateArm() {
        relicMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private double normalize(double number) {
        if (number > 1) {
            return 1;
        } else if (number < -1) {
            return -1;
        } else if (number < 0.37 && number >= 0) {
            return 0.37;
        } else if (number < 0) {
            return -0.1;
        } else {
            return number;
        }
    }

    private double reversenormalize(double p) {
        if (p > 1) {
            return 1;
        } else if (p < -1) {
            return -1;
        } else if (p > -0.37 && p <= 0) {
            return -0.37;
        } else if (p > 0) {
            return 0.1;
        } else {
            return p;
        }
    }

    private double extendnormalize(double n) {
        if (n > 1) {
            return 1;
        } else if (n < -1) {
            return -1;
        } else {
            return n;
        }
    }

}
