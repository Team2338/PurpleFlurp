package org.firstinspires.ftc.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.lib.PIDController;
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

    private final double kP = 0.001;
    private final double kI = 0;
    private final double kD = 0;

    private int position = 0;

    private DcMotor lift = RobotMap.lift;
    private Servo left = RobotMap.leftServo;
    private Servo right = RobotMap.rightServo;

    private PIDController liftPID = new PIDController(kP, kI, kD);

    private Lift() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setDirection(DcMotor.Direction.FORWARD);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update() {
        lift.setPower(liftPID.getOutput(lift.getCurrentPosition()));
    }

    public void raise() {
        position += 1;
        position = Range.clip(position, 0, 2);
        updateSetpoint();
    }

    public void lower() {
        position -= 1;
        position = Range.clip(position, 0, 2);
        updateSetpoint();
    }

    public void updateSetpoint() {
        if (position == 0) {
            setSetpoint(10);
        } else if (position == 1) {
            setSetpoint(1000);
        } else if (position == 2) {
            setSetpoint(1900);
        }
    }

    public void setSetpoint(double setpoint) {
        liftPID.setSetpoint(setpoint);
    }

    public int getPosition() {
        return lift.getCurrentPosition();
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
