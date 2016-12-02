package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Teacher on 10/28/2016.
 */

@TeleOp(name = "Team 2: Calibration", group = "Team 2")
//@Disabled
public class Team2Calibration extends OpMode {

    final int TARGET_POS = 1120; // TODO - Calibrate value

    final int LED_PORT = 3;

    final double BEACON_SERVO_HOME  = 0.5;

    int shooterOffset;

    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;

    DcMotor intakeMotor;
    DcMotor catapultMotor;

    Servo beaconServo;

    DeviceInterfaceModule cdim;
    ColorSensor sensorRGB;

    LightSensor lightSensor;

    String beaconColor;

    String teamColor;

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

        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        cdim.setDigitalChannelMode(LED_PORT, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelState(LED_PORT, false);

        sensorRGB = hardwareMap.colorSensor.get("color");

        lightSensor = hardwareMap.lightSensor.get("lightSensor");

        gamepad1.setJoystickDeadzone((float) 0.05);

        teamColor = "blue";
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

        if(sensorRGB.red() > sensorRGB.blue() && sensorRGB.red() > sensorRGB.green()) {
            beaconColor = "red";
        } else if(sensorRGB.blue() > sensorRGB.red() || sensorRGB.green() > sensorRGB.red()) {
            beaconColor = "blue";
        }

        if(gamepad1.a) {
            beaconServo.setPosition(beaconServo.getPosition() - 0.005);
        } else if(gamepad1.b) {
            beaconServo.setPosition(beaconServo.getPosition() + 0.005);
        }

        if(gamepad1.y) {
            if(teamColor.equals(beaconColor)) {
                while(beaconServo.getPosition() > 0) {
                    beaconServo.setPosition(beaconServo.getPosition() - 0.005);
                }
            } else {
                while(beaconServo.getPosition() < 1) {
                    beaconServo.setPosition(beaconServo.getPosition() + 0.005);
                }
            }
            long initTime = System.currentTimeMillis();
            while(System.currentTimeMillis() - initTime < 2000) { }
            beaconServo.setPosition(BEACON_SERVO_HOME);
        }

        telemetry.addData("Left front motor position", frontLeftMotor.getCurrentPosition());
        telemetry.addData("Right front motor position", frontRightMotor.getCurrentPosition());
        telemetry.addData("Light sensor", lightSensor.getLightDetected());
        telemetry.addData("Beacon servo", beaconServo.getPosition());
        telemetry.addData("Beacon color", beaconColor);
        telemetry.addData("Shooter pos", catapultMotor.getCurrentPosition() - shooterOffset);
        telemetry.update();
    }
}
