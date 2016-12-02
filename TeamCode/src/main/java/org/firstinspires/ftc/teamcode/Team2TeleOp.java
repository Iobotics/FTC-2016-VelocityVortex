package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Teacher on 9/28/2016.
 */

@TeleOp(name = "Team 2: TeleOp", group = "Team 2")
//@Disabled
public class Team2TeleOp extends OpMode {

    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor leftBackMotor;

    DcMotor intakeMotor;
    DcMotor catapultMotor;

    Servo beaconServo;

    DeviceInterfaceModule cdim;
    ColorSensor sensorRGB;

    final int CATAPULT_POWER = 1;
    final int CATAPULT_TICKS = 3 * 1120; // Three rotations

    final int LED_PORT = 3;


    final double BEACON_SERVO_LEFT  = 1.0;
    final double BEACON_SERVO_HOME  = 0.5;
    final double BEACON_SERVO_RIGHT = 0.0;

    int catapultOffset;
    String beaconColor;

    String teamColor;

    @Override
    public void init() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        rightBackMotor = hardwareMap.dcMotor.get("rightRear");

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        catapultMotor = hardwareMap.dcMotor.get("catapult");

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        cdim.setDigitalChannelMode(LED_PORT, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelState(LED_PORT, false);

        sensorRGB = hardwareMap.colorSensor.get("color");

        catapultMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        catapultOffset = catapultMotor.getCurrentPosition();

        beaconServo = hardwareMap.servo.get("beaconServo");
        beaconServo.setPosition(BEACON_SERVO_HOME);

        gamepad1.setJoystickDeadzone((float) 0.05);

        teamColor = "blue";
    }

    @Override
    public void loop() {
    	leftFrontMotor.setPower(gamepad1.left_stick_y);
        rightBackMotor.setPower(gamepad1.right_stick_y);
        rightFrontMotor.setPower(gamepad1.right_stick_y);
        leftBackMotor.setPower(gamepad1.left_stick_y);

        if(gamepad1.left_trigger > 0) {
            intakeMotor.setPower(-1.0);
        }
        else if(gamepad1.left_bumper) {
            intakeMotor.setPower(1.0);
        }
        else {
            intakeMotor.setPower(0);
        }

       /*if (gamepad1.a) {
            while((catapultMotor.getCurrentPosition() - catapultOffset) < CATAPULT_TICKS) {
                catapultMotor.setPower(.3);
                telemetry.addData("Ticks", catapultMotor.getCurrentPosition() - catapultOffset);
                telemetry.addData("catapult offset", catapultOffset);
                telemetry.addData("catapult ticks",catapultMotor.getCurrentPosition());
                telemetry.update();
            }
            catapultMotor.setPower(0);
            catapultOffset = catapultMotor.getCurrentPosition();
        }

        // Activates intake when B button is pressed
        if(gamepad1.b) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
		}
        */

        //right bumper for shooter

        if (gamepad1.right_bumper) {
            catapultMotor.setPower(1.0);
        } else {
            catapultMotor.setPower(0.0);
        }

        if(gamepad1.x) {
            while(catapultMotor.getCurrentPosition() - catapultOffset < CATAPULT_TICKS) {
                catapultMotor.setPower(1);
            }
            catapultMotor.setPower(0);
            catapultOffset = catapultMotor.getCurrentPosition();
        }

        if (gamepad1.right_trigger > .1) {
            while((catapultMotor.getCurrentPosition() - catapultOffset) < CATAPULT_TICKS){
                catapultMotor.setPower(CATAPULT_POWER);
            }
            catapultMotor.setPower(0);
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
                    beaconServo.setPosition(beaconServo.getPosition() - 0.01);
                }
            } else {
                while(beaconServo.getPosition() < 1) {
                    beaconServo.setPosition(beaconServo.getPosition() + 0.01);
                }
            }
            long initTime = System.currentTimeMillis();
            while(System.currentTimeMillis() - initTime < 2000) { }
            beaconServo.setPosition(BEACON_SERVO_HOME);
        }

        telemetry.addData("Ticks", catapultMotor.getCurrentPosition() - catapultOffset);
        telemetry.addData("Beacon servo", beaconServo.getPosition());
        telemetry.addData("Beacon color", beaconColor);
        telemetry.addData("Intake motor", intakeMotor.getPower());
        telemetry.update();
    }
}

