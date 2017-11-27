package org.firstinspires.ftc.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.lib.RobotMap;

/**
 *
 */

public class Lift {

    private static Lift instance = null;

    public static Lift getInstance() {
        if (instance == null) {
            instance = new Lift();
        }
        return instance;
    }

    private DcMotorEx lift = RobotMap.lift;
    private Servo left = RobotMap.leftServo;
    private Servo right = RobotMap.rightServo;

    private final double kP = 0.001;
    private final double kI = 0;
    private final double kD = 0;

    private int position = 0;

    private Lift() {
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        lift.setDirection(DcMotorEx.Direction.FORWARD);

        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lift.setPIDCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, new PIDCoefficients(kP, kI, kD));
    }

    public void raise(boolean isPressed) {
         position += isPressed ? 0 : 1;
         position = Range.clip(position, 0, 2);
    }

    public void lower(boolean isPressed) {
        position -= isPressed ? 0 : 1;
        position = Range.clip(position, 0, 2);
    }

    public void updateTargetPosition() {
        if (position == 0) {
            setTargetPosition(0, 1);
        } else if (position == 1) {
            setTargetPosition(-1000, 1);
        } else if (position == 2) {
            setTargetPosition(-1800, 1);
        }
    }

    public void setTargetPosition(int position, double power) {
//        lift.setTargetPosition(position);
        lift.setPower((position - lift.getCurrentPosition()) * kP);
    }

    public void openClaw() {
        left.setPosition(0);
        right.setPosition(0.2);
    }

    public void closeClaw() {
        left.setPosition(0.22);
        right.setPosition(0);
    }
}
