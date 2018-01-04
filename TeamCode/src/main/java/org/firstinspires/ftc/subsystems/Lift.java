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

    private DcMotor lift = RobotMap.lift;
    private Servo left = RobotMap.leftServo;
    private Servo right = RobotMap.rightServo;

	private final double kP = 0.002;
	private final double kI = 0;
	private final double kD = 0;

    private PIDController pid = new PIDController(kP, kI, kD);

	private int position = 0;

    private Lift() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setDirection(DcMotor.Direction.FORWARD);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid.setMaxIOutput(0.4);
    }

    public void update() {
    	lift.setPower(pid.getOutput(lift.getCurrentPosition()));
    }

	public void setSetpoint(double setpoint) {
    	pid.setSetpoint(setpoint);
    }

	public double getPosition() {
    	return lift.getCurrentPosition();
    }

	public boolean inTolerance() {
    	return pid.inTolerance(50);
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
            setSetpoint(7);
        } else if (position == 1) {
            setSetpoint(1300);
        } else if (position == 2) {
            setSetpoint(2600);
        }
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
