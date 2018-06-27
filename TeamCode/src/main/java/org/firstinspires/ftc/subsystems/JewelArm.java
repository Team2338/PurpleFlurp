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

    public static JewelArm getInstance() { //this is the instance that is used in other classes to create only one instance
        if (instance == null) {
            instance = new JewelArm();
        }
        return instance;
    }

    private Servo arm = RobotMap.arm; //RobotMap tells the robot where every piece of hardware is (another subsystem). This subsystem calls on RobotMap to find the servo. Other classes (TeleOp and autos) call on this subsystem
    public ColorSensor colorSensor = RobotMap.colorSensor; //same thing with the color sensor

    private JewelArm () {
    }

    public void armDown() { //public means this is open to all other classes in the library
        arm.setPosition(0); //this armDown means that we can type armDown instead of typing arm.setPosition every time we need to program in autos
    }                       //This makes a lot more sense with the Drivetrain subsystem. It is also why subsystems are used- to name things for simplicity

    public void armUp() {
        arm.setPosition(0.6);
    }

    public void armMidDown() {arm.setPosition(0.1);}
}
