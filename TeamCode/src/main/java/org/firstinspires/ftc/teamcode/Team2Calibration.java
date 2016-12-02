package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Teacher on 10/28/2016.
 */

@TeleOp(name = "Team 2: Calibration", group = "Team 2")
//@Disabled
public class Team2Calibration extends OpMode {

    final int TARGET_POS = 1120; // TODO - Calibrate value

    int shooterOffset;

    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;

    DcMotor intakeMotor;
    DcMotor catapultMotor;

    Servo beaconServo;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        catapultMotor = hardwareMap.dcMotor.get("catapult");

        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        catapultMotor.setDirection(DcMotor.Direction.REVERSE);
        catapultMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterOffset = catapultMotor.getCurrentPosition();

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gamepad1.setJoystickDeadzone((float) 0.05);
    }

    @Override
    public void loop() {
        frontLeftMotor.setPower(gamepad1.left_stick_y);
        backRightMotor.setPower(gamepad1.right_stick_y);
        frontRightMotor.setPower(gamepad1.right_stick_y);
        backLeftMotor.setPower(gamepad1.left_stick_y);

        // Left trigger to use shooter
        if(gamepad1.left_trigger > 0) {
            while((catapultMotor.getCurrentPosition() - shooterOffset) < TARGET_POS) {
                catapultMotor.setPower(1);
            }
            catapultMotor.setPower(0);
            shooterOffset = catapultMotor.getCurrentPosition();
        }

        telemetry.addData("Left front motor position", frontLeftMotor.getCurrentPosition());
        telemetry.addData("Right front motor position", frontRightMotor.getCurrentPosition());
        telemetry.addData("Shooter pos", catapultMotor.getCurrentPosition() - shooterOffset);
        telemetry.update();
    }
}
