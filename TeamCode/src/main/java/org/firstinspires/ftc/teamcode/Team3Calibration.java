package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Teacher on 10/28/2016.
 */

@TeleOp(name = "Team 3: Calibration", group = "Team 3")
//@Disabled
public class Team3Calibration extends OpMode {

    final double LEFT_SERVO_HOME = 0.45;
    final double RIGHT_SERVO_HOME = 0.55;
    final int TARGET_POS = 1120; // TODO - Calibrate value

    int shooterOffset;

    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor leftBackMotor;

    DcMotor intakeMotor;
    DcMotor shooterMotor;

    Servo rightBeaconServo;
    Servo leftBeaconServo;

    @Override
    public void init() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        rightBackMotor = hardwareMap.dcMotor.get("rightRear");

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        shooterMotor = hardwareMap.dcMotor.get("shooter");

        rightBeaconServo = hardwareMap.servo.get("rightBeacon");
        leftBeaconServo = hardwareMap.servo.get("leftBeacon");

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterOffset = shooterMotor.getCurrentPosition();

        leftBeaconServo.setPosition(0);
        rightBeaconServo.setPosition(0);

        gamepad1.setJoystickDeadzone((float) .1);
    }

    @Override
    public void loop() {

        // Left trigger to use shooter
        if(gamepad1.left_trigger > 0) {
            while((shooterMotor.getCurrentPosition() - shooterOffset) < TARGET_POS) {
                shooterMotor.setPower(gamepad1.left_trigger);
                telemetry.addData("Shooter position", shooterMotor.getCurrentPosition() - shooterOffset);
                telemetry.update();
            }
            shooterMotor.setPower(0);
            shooterOffset = shooterMotor.getCurrentPosition();
        }

        leftBeaconServo.setPosition(gamepad1.left_stick_x);
        rightBeaconServo.setPosition(gamepad1.right_stick_x);

        telemetry.addData("Left Servo Pos", leftBeaconServo.getPosition());
        telemetry.addData("Right Servo Pos", rightBeaconServo.getPosition());
        telemetry.addData("Shooter pos", shooterMotor.getCurrentPosition() - shooterOffset);
        telemetry.update();
    }
}
