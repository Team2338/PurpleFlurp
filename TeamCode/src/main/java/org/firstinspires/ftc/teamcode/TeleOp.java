/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Iterative Opmode")
@Disabled
public class TeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx rearLeft = null;
    private DcMotorEx rearRight = null;
    private DcMotorEx lift = null;

    private Servo leftServo = null;
    private Servo rightServo = null;
    private Servo arm = null;

    private BNO055IMU imu = null;

    private double maxSpeed = 0.9;
    private double driveP = 0;
    private double driveI = 0;
    private double driveD = 0;
    private double liftP = 0;
    private double liftI = 0;
    private double liftD = 0;

    private boolean padPressed = false;
    private int liftPosition = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Motor Setup
        frontLeft  = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight  = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft  = (DcMotorEx) hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight  = (DcMotorEx) hardwareMap.get(DcMotor.class, "rearRight");
        lift = (DcMotorEx) hardwareMap.get(DcMotor.class, "lift");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor PID
        PIDCoefficients drivePID = new PIDCoefficients(driveP, driveI, driveD);
        frontLeft.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, drivePID);
        frontRight.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, drivePID);
        rearLeft.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, drivePID);
        rearRight.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, drivePID);

        PIDCoefficients currentDrivePID = frontLeft.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDCoefficients liftPID = new PIDCoefficients(liftP, liftI, liftD);
        lift.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, liftPID);

        // Servo
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        arm = hardwareMap.get(Servo.class, "arm");

        // Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.addData("P, I, D (Drive)", "%.04f, %.04f, %.0f", currentDrivePID.p, currentDrivePID.i, currentDrivePID.d);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Movement
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double z = gamepad1.right_stick_x;

        frontLeft.setPower(normalize(x + y + z) * maxSpeed);
        frontRight.setPower(normalize(x - y + z) * maxSpeed);
        rearLeft.setPower(normalize(-x + y + z) * maxSpeed);
        rearRight.setPower(normalize(-x - y + z) * maxSpeed);

        // Lift Mechanism
        if (gamepad2.dpad_up && !padPressed) {
            raiseLift();
            padPressed = true;
        } else if (gamepad2.dpad_down && !padPressed) {
            lowerLift();
            padPressed = true;
        } else if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
            padPressed = false;
        }

        lift.setPower(0.5);

        // Claw Mechanism
        if (gamepad2.left_bumper) {
            leftServo.setPosition(0);
            rightServo.setPosition(0.2);
        } else if (gamepad2.right_bumper) {
            leftServo.setPosition(0.22);
            rightServo.setPosition(0);
        }

        // Telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void raiseLift() {
        liftPosition += liftPosition != 2 ? 1 : 0;
        moveLift();
    }

    private void lowerLift() {
        liftPosition -= liftPosition != 0 ? 1 : 0;
        moveLift();
    }

    private void moveLift() {
        switch (liftPosition) {
            case 0:
                lift.setTargetPosition(0);
            case 1:
                lift.setTargetPosition(-1000);
            case 2:
                lift.setTargetPosition(-1800);
        }
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
