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
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
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
    private int stage = 0;
    private int lastStage = -1;
    public static final String TAG = "Vuforia";

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AQzQCQj/////AAAAGZlcBiWnoE92snvweeDM1eY0IvsYZGPC/fahY6+LNXlrpjIDnd3l4vBZaLlI0xTgwVofau+yL+N0C/x1vYc8gRBPs6NpLp4e+l9DtF/sjw1sUP3eSQLIe6nxe1uILWzf9P9S+SI8TL+eZlntAZ/Jvqgo3JYiEOU1pXO9UHBcSxd9hUJFAI897emtj4Tn4rif0iPr53Pg3zm2kUVp44YsTqJ/DDMrhpBd6hXj/xwLIEj9zgetTFLFrBinVykQtKWLQuo+MVH4Whh7kXmu2UloHHtV3tp0pECZMqNeVbECuAg+gtZwcFjDbfA2pnEz3b+gVBe57ylm3QdQbllEpzilS7h0WXSj42u476b54eDa2Cbt";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
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
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Stage", stage);
        telemetry.addData("Position", movement.getPosition3());
        telemetry.update();

        if (stage != lastStage) {
            runtime.reset();
            movement.resetEncoders();
        }

        lastStage = stage;

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if (stage == 0) {
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);

                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    stage = 16;
                } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                    stage = 33;
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    stage = 1;
                }
            } else {
                stage = 1;
            }
        }
        if (stage == 1) {
        	lift.closeClaw(); // Grab the block before moving
        	stage = runtime.seconds() >= 1.2 ? 2 : 1; // Wait 0.5 seconds
		} else if (stage == 2) {
			lift.setSetpoint(1000); // Raise the lift with the now grabbed block
        	stage = runtime.seconds() >= 1.3 ? 3 : 2; // Wait for the lift to go up
		} else if (stage == 3) {
        	jewelArm.armDown(); // Lower color sensor
        	stage = runtime.seconds() >= 0.7 ? 4 : 3; // Wait 0.5 seconds
		} else if (stage == 4) {
			stage = hsvValues[0] > 120 && hsvValues[0] < 250 ? 5 : 6; // Measure hue and determine stage
		} else if (stage == 5) { // Blue detected
            movement.ForwardKnock();
            stage = runtime.seconds() >= 0.22 ? 7 : 5;
		} else if (stage == 6) { // Red detected (Not blue)
            movement.BackwardKnock();
            stage = runtime.seconds() >= 0.22 ? 8 : 6;
		} else if (stage == 7) { // Blue detected
            movement.StoptheMotor();
            jewelArm.armUp();
            stage = runtime.seconds() >= 0.7 ? 9 : 7;
        } else if (stage == 8) { // Red detected
            movement.StoptheMotor();
            jewelArm.armUp();
            stage = runtime.seconds() >= 0.7 ? 10 : 8;
        } else if (stage == 9) { //Blue detected
            movement.GetIntoBoxF();
            stage = runtime.seconds() >= 0.55 ? 11 : 9;
        } else if (stage == 10) { //Red detected
            movement.RampUp();
            stage = runtime.seconds() >= 1 ? 12 : 10;
        } else if (stage == 11) { //Blue detected
            movement.StoptheMotor();
            stage = runtime.seconds() >= 0.5 ? 13 : 11;
        } else if (stage == 12) { //Red detected
            movement.StoptheMotor();
            stage = runtime.seconds() >= 0.5 ? 13 : 12;
        } else if (stage == 13) { //Turn maybe?
            movement.TurnLeft();
            stage = runtime.seconds() >= 0.45 ? 14 : 13;
        } else if (stage == 14) {
            movement.StoptheMotor();
            stage = runtime.seconds() >= 0.3 ? 200 : 14;

            //THIS IS For RIGHT DETECTED
        } else  if (stage == 16) {
            lift.closeClaw(); // Grab the block before moving
            stage = runtime.seconds() >= 1.2 ? 17 : 16; // Wait 0.5 seconds
        } else if (stage == 17) {
            lift.setSetpoint(1000); // Raise the lift with the now grabbed block
            stage = runtime.seconds() >= 1.3 ? 18 : 17; // Wait for the lift to go up
        } else if (stage == 18) {
            stage = runtime.seconds() >= 0.1 ? 19 : 18;
        } else if (stage == 19) {
            jewelArm.armDown(); // Lower color sensor
            stage = runtime.seconds() >= 0.7 ? 20 : 19; // Wait 0.5 seconds
        } else if (stage == 20) {
            stage = hsvValues[0] > 120 && hsvValues[0] < 250 ? 21 : 22; // Measure hue and determine stage
        } else if (stage == 21) { // Blue detected
            movement.ForwardKnock();
            stage = runtime.seconds() >= 0.22 ? 23 : 21;
        } else if (stage == 22) { // Red detected (Not blue)
            movement.BackwardKnock();
            stage = runtime.seconds() >= 0.22 ? 24 : 22;
        } else if (stage == 23) { // Blue detected
            movement.StoptheMotor();
            jewelArm.armUp();
            stage = runtime.seconds() >= 0.7 ? 25 : 23;
        } else if (stage == 24) { // Red detected
            movement.StoptheMotor();
            jewelArm.armUp();
            stage = runtime.seconds() >= 0.7 ? 26 : 24;
        } else if (stage == 25) { //Blue detected
            movement.GetIntoBoxF();
            stage = runtime.seconds() >= 0.55 ? 27 : 25;
        } else if (stage == 26) { //Red detected
            movement.RampUp();
            stage = runtime.seconds() >= 1 ? 28 : 26;
        } else if (stage == 27) { //Blue detected
            movement.StoptheMotor();
            stage = runtime.seconds() >= 0.5 ? 29 : 27;
        } else if (stage == 28) { //Red detected
            movement.StoptheMotor();
            stage = runtime.seconds() >= 0.5 ? 29 : 28;
        } else if (stage == 29) { //Turn maybe?
            movement.TurnRight();
            stage = runtime.seconds() >= 1 ? 30 : 29;
        } else if (stage == 30) {
            movement.StoptheMotor();
            stage = runtime.seconds() >= 0.3 ? 31 : 30;
        } else if (stage == 31) {
            movement.BackwardKnock();
            stage = runtime.seconds() >= 0.7 ? 32 : 31;
        } else if (stage == 32) {
            movement.StoptheMotor();
            stage = runtime.seconds() >= 0.2 ? 200 : 32;


            //This is for LEFT Detected
        }else if (stage == 33) {
            lift.closeClaw(); // Grab the block before moving
            stage = runtime.seconds() >= 1.2 ? 34 : 33; // Wait 0.5 seconds
        } else if (stage == 34) {
            lift.setSetpoint(1000); // Raise the lift with the now grabbed block
            stage = runtime.seconds() >= 1.3 ? 35 : 34; // Wait for the lift to go up
        } else if (stage == 35) {
            stage = runtime.seconds() >= 0.1 ? 36 : 35;
        } else if (stage == 36) {
            jewelArm.armDown(); // Lower color sensor
            stage = runtime.seconds() >= 0.7 ? 37 : 36; // Wait 0.5 seconds
        } else if (stage == 37) {
            stage = hsvValues[0] > 120 && hsvValues[0] < 250 ? 38 : 39; // Measure hue and determine stage
        } else if (stage == 38) { // Blue detected
            movement.ForwardKnock();
            stage = runtime.seconds() >= 0.22 ? 40 : 38;
        } else if (stage == 39) { // Red detected (Not blue)
            movement.BackwardKnock();
            stage = runtime.seconds() >= 0.22 ? 41 : 39;
        } else if (stage == 40) { // Blue detected
            movement.StoptheMotor();
            jewelArm.armUp();
            stage = runtime.seconds() >= 0.7 ? 42 : 40;
        } else if (stage == 41) { // Red detected
            movement.StoptheMotor();
            jewelArm.armUp();
            stage = runtime.seconds() >= 0.7 ? 43 : 41;
        } else if (stage == 42) { //Blue detected
            movement.GetIntoBoxF();
            stage = runtime.seconds() >= 0.55 ? 44 : 42;
        } else if (stage == 43) { //Red detected
            movement.RampUp();
            stage = runtime.seconds() >= 1 ? 45 : 43;
        } else if (stage == 44) { //Blue detected
            movement.StoptheMotor();
            stage = runtime.seconds() >= 0.5 ? 46 : 44;
        } else if (stage == 45) { //Red detected
            movement.StoptheMotor();
            stage = runtime.seconds() >= 0.5 ? 46 : 45;
        } else if (stage == 46) { //Turn maybe?
            movement.TurnRight();
            stage = runtime.seconds() >= 1.5 ? 47 : 46;
        } else if (stage == 47) {
            movement.StoptheMotor();
            stage = runtime.seconds() >= 0.3 ? 48 : 47;
        } else if (stage == 48) {
            movement.BackwardKnock();
            stage = runtime.seconds() >= 0.7 ? 49 : 48;
        } else if (stage == 49) {
            movement.StoptheMotor();
            stage = runtime.seconds() >= 0.2 ? 200 : 49;
        }








        else if (stage == 200) {
            lift.setSetpoint(10);
        }

        lift.update();
//        movement.updateDrive();
//        movement.updateRotate();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
