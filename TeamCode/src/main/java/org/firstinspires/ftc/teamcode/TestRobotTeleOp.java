package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Teacher on 9/28/2016.
 */
//Noting Code, not for code
    //When you turn on the robot the FTC app will automatically open.
    //Start+A button on remote to start TeleOp
@TeleOp(name="TestRobot: Teleop", group="TestRobot") //Alt enter to import code
//@Disabled
public class TestRobotTeleOp extends OpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    //Servo wing;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");
       // wing = hardwareMap.servo.get(""); // TODO - Name hardware

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
        if(gamepad1.left_bumper) {

        }
    }
}
