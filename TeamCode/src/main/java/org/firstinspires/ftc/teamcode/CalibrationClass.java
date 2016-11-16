package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Teacher on 10/28/2016.
 */

@TeleOp(name = "Test: Calibration", group = "Test")
@Disabled
public class CalibrationClass extends OpMode {

    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor leftBackMotor;

    DcMotor intakeMotor;
    DcMotor shooterMotor;
    DcMotor catapultMotor;

    //Servo rightBeaconServo;
    //Servo leftBeaconServo;

    int leftOffset;
    int rightOffset;

    @Override
    public void init() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        rightBackMotor = hardwareMap.dcMotor.get("rightRear");

        //intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        //shooterMotor = hardwareMap.dcMotor.get("shooter");

        //rightBeaconServo = hardwareMap.servo.get("rightBeacon");
        //leftBeaconServo = hardwareMap.servo.get("leftBeacon");
        catapultMotor = hardwareMap.dcMotor.get("catapult");


        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        //shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //catapultMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftOffset = leftFrontMotor.getCurrentPosition();
        rightOffset = rightFrontMotor.getCurrentPosition();

        //leftBeaconServo.setPosition(.5);  // TODO - Calibrate value
        //rightBeaconServo.setPosition(.5); // TODO - Calibrate value

        gamepad1.setJoystickDeadzone((float) .1);
    }

    @Override
    public void loop() {
        //telemetry.addData("Left Servo Pos", leftBeaconServo.getPosition());
        //telemetry.addData("Right Servo Pos", rightBeaconServo.getPosition());
        telemetry.addData("Left Front Position", leftFrontMotor.getCurrentPosition() - leftOffset);
        telemetry.addData("Left Rear Position", leftBackMotor.getCurrentPosition() - leftOffset);
        telemetry.addData("Right Front Position", rightFrontMotor.getCurrentPosition() - rightOffset);
        telemetry.addData("Right Rear Position", rightBackMotor.getCurrentPosition() - rightOffset);

        //telemetry.addData("Catapult Position", shooterMotor.getCurrentPosition());

        telemetry.update();
    }
}
