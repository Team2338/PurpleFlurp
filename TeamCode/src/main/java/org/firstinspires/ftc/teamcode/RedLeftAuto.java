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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.lib.RobotMap;
import org.firstinspires.ftc.subsystems.Drivetrain;
import org.firstinspires.ftc.subsystems.JewelArm;
import org.firstinspires.ftc.subsystems.Lift;

import static java.lang.Thread.sleep;

/*
 * This is an Iterative Autonomous OpMode for the left position on the
 * red alliance.
 */
@Autonomous(name = "RedLeftAuto", group = "Auto")
//@Disabled                            // Comment this out to add to the opmode list
public class RedLeftAuto extends OpMode {

    /**
     * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
     * It has a light/distance (range) sensor.  It also has an RGB color sensor.
     * The light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
     * or closer will display the same value for distance/light detected.
     *
     * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
     * you can treat the sensor as two separate sensors that share the same name in your op mode.
     *
     * In this example, we represent the detected color by a hue, saturation, and value color
     * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
     * color of the screen to match the detected color.
     *
     * In this example, we  also use the distance sensor to display the distance
     * to the target object.  Note that the distance sensor saturates at around 2" (5 cm).
     *
     */
    private JewelArm jewelArm;
    private Lift lift;
    private Drivetrain movement;

    private ElapsedTime runtime = new ElapsedTime();
    private boolean firstRun = true;
    private boolean blue = false;
    private int stage = 0;
    private int lastStage = 0;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        RobotMap.init(hardwareMap);
        jewelArm = JewelArm.getInstance();
        lift = Lift.getInstance();
        movement = Drivetrain.getInstance();
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
        resetStartTime();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (jewelArm.colorSensor.red() * SCALE_FACTOR),
                (int) (jewelArm.colorSensor.green() * SCALE_FACTOR),
                (int) (jewelArm.colorSensor.blue() * SCALE_FACTOR),
                hsvValues);

        // send the info back to driver station using telemetry function.
        //telemetry.addData("Distance (cm)",
        //String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha", jewelArm.colorSensor.alpha());
        telemetry.addData("Red  ", jewelArm.colorSensor.red());
        telemetry.addData("Green", jewelArm.colorSensor.green());
        telemetry.addData("Blue ", jewelArm.colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();

        if (getRuntime() < 0.5) {
            lift.closeClaw();
        }

        if (getRuntime() < 1) {
            lift.setSetpoint(1000);               //SERIOUSLY, CHECK WHAT I DID!!!!!!!!
        }

        lift.update();

        if (getRuntime() < 1.4) {
            jewelArm.armDown();
        }

        if (hsvValues[0] > 130 && hsvValues[0] < 250 && firstRun && getRuntime() > 2) {
            blue = true;
            firstRun = false;
        } else {
            blue = false;
            firstRun = false;
        }


        if (blue) {
            if (getRuntime() > 3 && getRuntime() < 3.2) {
                movement.ForwardKnock();
            }

            if (getRuntime() > 3.3) {
                movement.StoptheMotor();
            }

            if (getRuntime() > 3.4) {
                jewelArm.armUp();
            }

            if (getRuntime() > 5 && getRuntime() < 5.7) {
                movement.GetIntoBox();
            }

            if (getRuntime() > 5.7) {
                movement.StoptheMotor();
            }

        } else {

            if (getRuntime() > 3 && getRuntime() < 3.2) {
                movement.BackwardKnock();
            }

            if (getRuntime() > 3.3) {
                movement.StoptheMotor();
            }

            if (getRuntime() > 3.4) {
                jewelArm.armUp();
            }

            if (getRuntime() > 5 && getRuntime() < 6.2) {
                movement.RampUp();
            }

            if (getRuntime() > 6.2) {
                movement.StoptheMotor();
            }
        }

        if (stage != lastStage) runtime.reset();

        if (stage == 0) {
        	lift.closeClaw(); // Grab the block before moving
        	stage = runtime.seconds() >= 0.5 ? 1 : 0; // Wait 0.5 seconds
		} else if (stage == 1) {
			lift.raise(); // Raise the lift with the now grabbed block
        	stage = lift.inTolerance() ? 2 : 1; // Wait until the lift is within tolerance
		} else if (stage == 2) {
        	jewelArm.armDown(); // Lower color sensor
        	stage = runtime.seconds() >= 0.5 ? 3 : 2; // Wait 0.5 seconds
		} else if (stage == 3) {
			stage = hsvValues[0] > 130 && hsvValues[0] < 250 ? 4 : 5; // Measure hue and determine stage
		} else if (stage == 4) { // Blue detected

		} else if (stage == 5) { // Red detected (Not blue)

		}

		lastStage = stage;
        lift.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}