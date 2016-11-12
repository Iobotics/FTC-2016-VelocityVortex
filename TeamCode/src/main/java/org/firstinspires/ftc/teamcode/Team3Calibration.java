package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Teacher on 10/28/2016.
 */

@TeleOp(name = "Team 3: Calibration", group = "Team 3")
//@Disabled
public class Team3Calibration extends OpMode {

	// Constants //
    final int SHOOTER_ROTATION 	  = 765;
    final double LEFT_SERVO_MIN	  = 0.132;
    final double RIGHT_SERVO_MIN  = 0;
    final double LEFT_SERVO_HOME  = 0.74;
    final double RIGHT_SERVO_HOME = 0.55;
    final boolean LED_STATE 	  = false;

    // Member variables //
    int shooterOffset;

    boolean leftBeaconButton = false;
    boolean rightBeaconButton = false;

    // Hardware declarations //
    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor leftBackMotor;

    DcMotor intakeMotor;
    DcMotor shooterMotor;

    Servo rightBeaconServo;
    Servo leftBeaconServo;

    ColorSensor sensorRGB;
    DeviceInterfaceModule cdim;

    @Override
    public void init() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        rightBackMotor = hardwareMap.dcMotor.get("rightRear");

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        leftBeaconServo = hardwareMap.servo.get("leftBeacon");
        rightBeaconServo = hardwareMap.servo.get("rightBeacon");

        leftBeaconServo.scaleRange(LEFT_SERVO_MIN, LEFT_SERVO_HOME);
        rightBeaconServo.scaleRange(RIGHT_SERVO_MIN, RIGHT_SERVO_HOME);

        leftBeaconServo.setPosition(0);
        rightBeaconServo.setPosition(0);

        shooterMotor = hardwareMap.dcMotor.get("shooter");
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterOffset = shooterMotor.getCurrentPosition();

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        sensorRGB = hardwareMap.colorSensor.get("color");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");

        cdim.setDigitalChannelMode(5, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelState(5, LED_STATE);
    }

    @Override
    public void loop() {
        leftFrontMotor.setPower(gamepad1.left_stick_y);
        leftBackMotor.setPower(gamepad1.left_stick_y);
        rightFrontMotor.setPower(gamepad1.right_stick_y);
        rightBackMotor.setPower(gamepad1.right_stick_y);

        if(gamepad1.right_trigger > 0) {
            intakeMotor.setPower(1);
        }
        else if(gamepad1.right_bumper) {
            intakeMotor.setPower(-1);
        }
        else {
            intakeMotor.setPower(0);
        }

        if(gamepad1.left_trigger > 0) {
            while(getShooterPosition() < SHOOTER_ROTATION) {
                shooterMotor.setPower(1);
            }
            shooterMotor.setPower(0);
            shooterOffset = shooterMotor.getCurrentPosition();
        }
        else if(gamepad1.left_bumper) {
            shooterMotor.setPower(1);
        }
        else {
            shooterMotor.setPower(0);
        }

        if(gamepad1.a && !rightBeaconButton) {
            rightBeaconServo.setPosition((rightBeaconServo.getPosition() < 0.2) ? 1 : 0);
            rightBeaconButton = true;
        } else if(!gamepad1.a) rightBeaconButton = false;

        if(gamepad1.x && !leftBeaconButton) {
            leftBeaconServo.setPosition((leftBeaconServo.getPosition() < 0.2) ? 1 : 0);
            leftBeaconButton = true;
        } else if(!gamepad1.x) leftBeaconButton = false;

        telemetry.addData("Left trigger", gamepad1.left_trigger > 0);
        telemetry.addData("Left bumper", gamepad1.left_bumper);
        telemetry.addData("Red", sensorRGB.red());
        telemetry.addData("Green", sensorRGB.green());
        telemetry.addData("Blue", sensorRGB.blue());
        telemetry.addData("Shooter pos", shooterMotor.getCurrentPosition() - shooterOffset);
        telemetry.update();
    }

    private int getShooterPosition() {
        return shooterMotor.getCurrentPosition() - shooterOffset;
    }
}
