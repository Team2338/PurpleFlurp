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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.lib.RobotMap;
import org.firstinspires.ftc.subsystems.Drivetrain;
import org.firstinspires.ftc.subsystems.JewelArm;
import org.firstinspires.ftc.subsystems.Lift;
import org.firstinspires.ftc.subsystems.Relic;

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

@TeleOp(name="DefaultTeleOp", group="TeleOp")
//@Disabled
public class DefaultTeleOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Drivetrain drivetrain; //again naming variables (see RedLeft)
    private Lift lift;
    private JewelArm jewelArm;
    private Relic relic;

    private boolean padPressed = false; //Booleans are entities that can be true or false and are used if a button is pressed
    private boolean rPressed = false;
    private boolean turnon = false;
    private boolean reverseC = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        RobotMap.init(hardwareMap);
        drivetrain = Drivetrain.getInstance();
        lift = Lift.getInstance();
        jewelArm = JewelArm.getInstance();
        relic = Relic.getInstance();


        // Telemetry
        telemetry.addData("Status", "Initialized");
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
        drivetrain.mecanumDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x); //calls on subsystem drivetrain and sets x,y,x

        //Relic
        if (gamepad1.a) { //if A is pressed the boolean turnon is set as true if it is false
            if (!turnon) {
                turnon = true;
            }
        }

        if (turnon) { //turnon is when the relic arm is on. If so reverseC sets the direction of the motor spinning
            if (!reverseC) {
                relic.activateArm();
                relic.relicDrive(-gamepad2.left_stick_y);
            } else if (reverseC) {
                relic.activateArm();
                relic.relicReverse(-gamepad2.left_stick_y);
            }
        }

        if (relic.getPosition() > 1100 && relic.getPosition() < 1820 ) { //uses encoder positions to automatically set the direction of the motor of the relicarm
            if (!reverseC) {
                reverseC = true;
            }
        } else if (relic.getPosition() > -3 && relic.getPosition() < 600) {
            if (reverseC) {
                reverseC = false;
            }
        }


        relic.relicExtension(-gamepad2.right_stick_y); //controls the relic extension using this right stick

        // Lift Mechanism
        if (gamepad2.dpad_up) {
            if (!padPressed) lift.raise(); //this is how the lift raises in increments, using booleans and another if statement
            padPressed = true;
        } else if (gamepad2.dpad_down) {
            if (!padPressed) lift.lower();
            padPressed = true;
        } else if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
            padPressed = false;
        }

        //Backup Lift Mechanism
//        if (gamepad2.dpad_down) {
//            lift.powerUp();
//        } else if (gamepad2.dpad_up) {
//            lift.powerDown();
//        } else {
//            lift.noPower();
//        }

        lift.update();

        // Claw Mechanism
        if (gamepad2.left_bumper) {
            lift.openClaw();
        } else if (gamepad2.right_bumper) {
            lift.closeClaw();
        }

        if (gamepad1.left_bumper) {
            if (!rPressed) {
                relic.lower();
                rPressed = true;
            }
        } else if (gamepad1.right_bumper) {
            if (!rPressed) {
                relic.raise();
                rPressed = true;
            }
        } else if (!gamepad1.left_bumper && !gamepad1.right_bumper) { //same as the lift mechanism to create claw increments
            rPressed = false;
        }

        if (gamepad2.b) {
            jewelArm.armUp();
        }

        // Telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Lift Position: ", lift.getPosition());
        telemetry.addData("Relic Position: ", relic.getPosition()); //displays certain things on the phone
//        telemetry.addData("frontLeft", drivetrain.getPosition());
//        telemetry.addData("frontRight", drivetrain.getPosition2());
//        telemetry.addData("rearRight", drivetrain.getPosition4());
//        telemetry.addData("rearLeft", drivetrain.getPosition3());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
