package org.firstinspires.ftc.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
   //private Servo relicLeft = RobotMap.relicLeft;
    //private Servo relicRight = RobotMap.relicRight;
    private DcMotor relicMotor = RobotMap.relicMotor;


    private Relic() {
        relicMotor.setDirection(DcMotor.Direction.FORWARD);
        relicMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void relicDrive(double q) {          //Check if this works
        relicMotor.setPower(q);
    }

    public void relicClose() {
        //relicLeft.setPosition(0);
        //relicRight.setPosition(0);
    }

    public void relicOpen() {
        //relicLeft.setPosition(0.2);
        //relicRight.setPosition(0.2);
    }

}
