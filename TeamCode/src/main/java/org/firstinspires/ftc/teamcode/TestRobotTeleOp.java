package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name="TestRobot: Teleop", group="TestRobot")
//@Disabled
public class TestRobotTeleOp extends OpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE); //Motors will normally start at way that is not to good
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        gamepad1.setJoystickDeadzone((float).05); //Joystick going
    }

    @Override
    public void loop() {
        frontLeftMotor.setPower(gamepad1.left_stick_y); //Tank Drive
        frontRightMotor.setPower(gamepad1.right_stick_y);
        backLeftMotor.setPower(gamepad1.left_stick_y);
        backRightMotor.setPower(gamepad1.right_stick_y);

        telemetry.addData("Front Left Power", frontLeftMotor.getPowerFloat()); //String is objective and getting power of motor
        telemetry.update(); //Adds data to phone
    }
}
