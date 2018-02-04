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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.lib.RobotMap;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.subsystems.Drivetrain;
import org.firstinspires.ftc.subsystems.JewelArm;
import org.firstinspires.ftc.subsystems.Lift;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="RedRight", group = "Auto")
//@Disabled
public class RedRight extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    private JewelArm jewelArm;
    private Lift lift;
    private Drivetrain movement;
    private ElapsedTime runtime = new ElapsedTime();

    private int stage = 15;
    private int lastStage = -1;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    //DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {

        RobotMap.init(hardwareMap);
        jewelArm = JewelArm.getInstance();
        lift = Lift.getInstance();
        movement = Drivetrain.getInstance();

        //distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");

        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AQzQCQj/////AAAAGZlcBiWnoE92snvweeDM1eY0IvsYZGPC/fahY6+LNXlrpjIDnd3l4vBZaLlI0xTgwVofau+yL+N0C/x1vYc8gRBPs6NpLp4e+l9DtF/sjw1sUP3eSQLIe6nxe1uILWzf9P9S+SI8TL+eZlntAZ/Jvqgo3JYiEOU1pXO9UHBcSxd9hUJFAI897emtj4Tn4rif0iPr53Pg3zm2kUVp44YsTqJ/DDMrhpBd6hXj/xwLIEj9zgetTFLFrBinVykQtKWLQuo+MVH4Whh7kXmu2UloHHtV3tp0pECZMqNeVbECuAg+gtZwcFjDbfA2pnEz3b+gVBe57ylm3QdQbllEpzilS7h0WXSj42u476b54eDa2Cbt";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */

        // send the info back to driver station using telemetry function.

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {

            Color.RGBToHSV((int) (jewelArm.colorSensor.red() * SCALE_FACTOR),
                    (int) (jewelArm.colorSensor.green() * SCALE_FACTOR),
                    (int) (jewelArm.colorSensor.blue() * SCALE_FACTOR),
                    hsvValues);

            //telemetry.addData("Distance (cm)",
            //        String.format(Locale.US, "%.02f", distanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Stage", stage);
            telemetry.addData("Position", movement.getPosition3());
            telemetry.update();

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (stage != lastStage) {
                runtime.reset();
                //movement.resetEncoders();
            }

            lastStage = stage;

//            if (stage == 0) {
//                stage = runtime.seconds() > 2 ? 1 : 0;
//            } else if (stage == 1) {
//
//                /* Found an instance of the template. In the actual game, you will probably
//                 * loop until this condition occurs, then move on to act accordingly depending
//                 * on which VuMark was visible. */
//                telemetry.addData("VuMark", "%s visible", vuMark);
//
//                if (vuMark == RelicRecoveryVuMark.RIGHT) {
//                    stage = 15;
//                } else if (vuMark == RelicRecoveryVuMark.LEFT) {
//                    stage = 90;
//                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
//                    stage = 50;
//                } else if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
//                    stage = 5;
//                    //Backup stage # is here
//                }
//            }

            //CODE FOR RIGHT BLOCK
            if (stage == 15) {
                lift.setSetpoint(20);
                stage = runtime.seconds() >= 1 ? 16 : 15;
            } else if (stage == 16) {
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
                stage = runtime.seconds() >= 0.17 ? 24 : 22;
            } else if (stage == 23) { // Blue detected
                movement.StoptheMotor();
                jewelArm.armUp();
                stage = runtime.seconds() >= 1 ? 25 : 23;
            } else if (stage == 24) { // Red detected
                movement.StoptheMotor();
                jewelArm.armUp();
                stage = runtime.seconds() >= 1 ? 26 : 24;
                //WORK ON THESE VALUES (RIGHT)
            } else if (stage == 25) { //Blue detected
                movement.GetIntoBoxF();
                stage = runtime.seconds() >= 0.5 ? 27 : 25;
            } else if (stage == 26) { //Red detected
                movement.RampUp();
                stage = runtime.seconds() >= 0.95 ? 27 : 26;
            } else if (stage == 27) {
                movement.StoptheMotor();
                stage = runtime.seconds() >= 0.4 ? 28 : 27;
            } else if (stage == 28) {
                movement.mecanumDrive(0, 0, -0.5);
                stage = runtime.seconds() >= 2.1 ? 29 : 28;
            } else if (stage == 29) {
                movement.StoptheMotor();
                stage = runtime.seconds() >= 1 ? 30 : 29;
            } else if (stage == 30) {
                movement.BackwardKnock();
                stage = runtime.seconds() >= 0.2 ? 31 : 30;
            } else if (stage == 31) {
                movement.StoptheMotor();
                stage = runtime.seconds() >= 0.2 ? 32 : 31;
            } else if (stage == 32) {
                movement.mecanumDrive(0, 0, 0.5);
                stage = runtime.seconds() >= 0.2 ? 33 : 32;
            } else if (stage == 33) {
                movement.StoptheMotor();
                stage = runtime.seconds() >= 0.3 ? 34 : 33;
            } else if (stage == 34) {
                movement.VerytinyF();
                stage = runtime.seconds() >= 2 ? 35 : 34;
            } else if (stage == 35) {
                movement.StoptheMotor();
                stage = runtime.seconds() >= 1 ? 200 : 35;
//
//
//                //Code for the CENTER Block
//            } else if (stage == 50) {
//                lift.setSetpoint(-10);
//                stage = runtime.seconds() >= 1 ? 51 : 50;
//            } else if (stage == 51) {
//                lift.closeClaw(); // Grab the block before moving
//                stage = runtime.seconds() >= 1.2 ? 52 : 51; // Wait 0.5 seconds
//            } else if (stage == 52) {
//                lift.setSetpoint(-1000); // Raise the lift with the now grabbed block
//                stage = runtime.seconds() >= 1.3 ? 53 : 52; // Wait for the lift to go up
//            } else if (stage == 53) {
//                stage = runtime.seconds() >= 0.1 ? 54 : 53;
//            } else if (stage == 54) {
//                jewelArm.armDown(); // Lower color sensor
//                stage = runtime.seconds() >= 0.7 ? 55 : 54; // Wait 0.5 seconds
//            } else if (stage == 55) {
//                stage = hsvValues[0] > 120 && hsvValues[0] < 250 ? 56 : 57; // Measure hue and determine stage
//            } else if (stage == 56) { // Blue detected
//                movement.ForwardKnock();
//                stage = runtime.seconds() >= 0.22 ? 58 : 56;
//            } else if (stage == 57) { // Red detected (Not blue)
//                movement.BackwardKnock();
//                stage = runtime.seconds() >= 0.22 ? 59 : 57;
//            } else if (stage == 58) { // Blue detected
//                movement.StoptheMotor();
//                jewelArm.armUp();
//                stage = runtime.seconds() >= 1 ? 60 : 58;
//            } else if (stage == 59) { // Red detected
//                movement.StoptheMotor();
//                jewelArm.armUp();
//                stage = runtime.seconds() >= 1 ? 61 : 59;
//            } else if (stage == 60) { //Blue detected
//                movement.GetIntoBoxF();
//                stage = runtime.seconds() >= 0.6 ? 62 : 60;
//            } else if (stage == 61) { //Red detected
//                movement.RampUp();
//                stage = runtime.seconds() >= 0.9 ? 62 : 61;
//            } else if (stage == 62) {
//                movement.StoptheMotor();
//                stage = runtime.seconds() >= 0.4 ? 63 : 62;
//            } else if (stage == 63) {
//                movement.mecanumDrive(0, 0, -0.5);
//                stage = runtime.seconds() >= 1.2 ? 64 : 63;
//            } else if (stage == 64) {
//                movement.StoptheMotor();
//                stage = runtime.seconds() >= 1 ? 65 : 64;
//            } else if (stage == 65) {
//                movement.ForwardKnock();
//                stage = runtime.seconds() >= 0.5 ? 66 : 65;
//            } else if (stage == 66) {
//                movement.StoptheMotor();
//                stage = runtime.seconds() >= 0.2 ? 67 : 66;
//            } else if (stage == 67) {
//                movement.mecanumDrive(0, 0, 0.5);
//                stage = runtime.seconds() >= 1.2 ? 68 : 67;
//            } else if (stage == 68) {
//                movement.StoptheMotor();
//                stage = runtime.seconds() >= 0.4 ? 69 : 68;
//            } else if (stage == 69) {
//                movement.VerytinyF();
//                stage = runtime.seconds() >= 1 ? 200 : 69;
//
//
//                //Code for LEFT Block
//            } else if (stage == 90) {
//                lift.setSetpoint(-10);
//                stage = runtime.seconds() >= 1 ? 91 : 90;
//            } else if (stage == 91) {
//                lift.closeClaw(); // Grab the block before moving
//                stage = runtime.seconds() >= 1.2 ? 92 : 91; // Wait 0.5 seconds
//            } else if (stage == 92) {
//                lift.setSetpoint(-1000); // Raise the lift with the now grabbed block
//                stage = runtime.seconds() >= 1.3 ? 93 : 92; // Wait for the lift to go up
//            } else if (stage == 93) {
//                stage = runtime.seconds() >= 0.1 ? 94 : 93;
//            } else if (stage == 94) {
//                jewelArm.armDown(); // Lower color sensor
//                stage = runtime.seconds() >= 0.7 ? 95 : 94; // Wait 0.5 seconds
//            } else if (stage == 95) {
//                stage = hsvValues[0] > 120 && hsvValues[0] < 250 ? 96 : 97; // Measure hue and determine stage
//            } else if (stage == 96) { // Blue detected
//                movement.ForwardKnock();
//                stage = runtime.seconds() >= 0.22 ? 98 : 96;
//            } else if (stage == 97) { // Red detected (Not blue)
//                movement.BackwardKnock();
//                stage = runtime.seconds() >= 0.22 ? 99 : 97;
//            } else if (stage == 98) { // Blue detected
//                movement.StoptheMotor();
//                jewelArm.armUp();
//                stage = runtime.seconds() >= 1 ? 100 : 98;
//            } else if (stage == 99) { // Red detected
//                movement.StoptheMotor();
//                jewelArm.armUp();
//                stage = runtime.seconds() >= 1 ? 101 : 99;
//            } else if (stage == 100) { //Blue detected
//                movement.GetIntoBoxF();
//                stage = runtime.seconds() >= 0.9 ? 102 : 100;
//            } else if (stage == 101) { //Red detected
//                movement.RampUp();
//                stage = runtime.seconds() >= 1.2 ? 102 : 101;
//            } else if (stage == 102) {
//                movement.StoptheMotor();
//            } else if (stage == 103) {
//                movement.mecanumDrive(0, 0, .5);
//                stage = runtime.seconds() >= 1.2 ? 104 : 103;
//            } else if (stage == 104) {
//                movement.StoptheMotor();
//                stage = runtime.seconds() >= 1 ? 105 : 104;
//            } else if (stage == 105) {
//                stage = runtime.seconds() >= 0.2 ? 106 : 105;
//            } else if (stage == 106) {
//                movement.StoptheMotor();
//                stage = runtime.seconds() >= 0.3 ? 107 : 106;
//            } else if (stage == 107) {
//                movement.ForwardKnock();
//                stage = runtime.seconds() >= 0.9 ? 108 : 107;
//            } else if (stage == 108) {
//                movement.StoptheMotor();
//                stage = runtime.seconds() >= 0.2 ? 109 : 108;
//            } else if (stage == 109) {
//                movement.mecanumDrive(0, 0, 0.5);
//                stage = runtime.seconds() >= 0.8 ? 110 : 109;
//            } else if (stage == 110) {
//                movement.StoptheMotor();
//                stage = runtime.seconds() >= 0.2 ? 111 : 110;
//            } else if (stage == 111) {
//                movement.VerytinyF();
//                stage = runtime.seconds() >= 1 ? 200 : 111;




                //Ending program for all functions
            } else if (stage == 200) {
                lift.setSetpoint(20);
                movement.StoptheMotor();
            }

                    /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
//                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
//                telemetry.addData("Pose", format(pose));
//
//                /* We further illustrate how to decompose the pose into useful rotational and
//                 * translational components */
//                if (pose != null) {
//                    VectorF trans = pose.getTranslation();
//                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//
//                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
//                    double tX = trans.get(0);
//                    double tY = trans.get(1);
//                    double tZ = trans.get(2);
//
//                    // Extract the rotational components of the target relative to the robot
//                    double rX = rot.firstAngle;
//                    double rY = rot.secondAngle;
//                    double rZ = rot.thirdAngle;


            telemetry.update();
            lift.update();
        }
    }



    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
