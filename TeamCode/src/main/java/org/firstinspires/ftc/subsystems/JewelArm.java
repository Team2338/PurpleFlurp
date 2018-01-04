package org.firstinspires.ftc.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.lib.RobotMap;

/**
 *
 */

public class JewelArm {

    private static JewelArm instance = null;

    public static JewelArm getInstance() {
        if (instance == null) {
            instance = new JewelArm();
        }
        return instance;
    }

    private Servo arm = RobotMap.arm;
    public ColorSensor colorSensor = RobotMap.colorSensor;

    private JewelArm () {
    }

    public void armDown() {
        arm.setPosition(0);
    }

    public void armUp() {
        arm.setPosition(0.6);
    }

    public void armMidDown() {arm.setPosition(0.1);}
}
