package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name = "Team 1 Red: Autonomous", group = "Team 1 Red")
//@Disabled
public class Team1AutoRed extends OpMode {

    final int    ENCODER_TICKS_PER_REV = 1120; // Neverest 40
    final int    WHEEL_DIAMETER        = 6;
    final double INCHES_PER_TICK       = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV; // INCHES / REV
    final double LEFT_OFFSET = 0.25;
    final int TICK_OFFSET = 700;
    final int CATAPULT_ROTATIONS = 3 * ENCODER_TICKS_PER_REV;
    final int INTAKE_TICKS = ENCODER_TICKS_PER_REV * 1;

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    DcMotor catapultMotor;
    DcMotor intakeMotor;

    ColorSensor colorSensor;

    Servo leftServo;
    Servo rightServo;

    int catapultOffset;
    int rightMotorOffset;
    int leftMotorOffset;

    int targetRotations;
    int intakeOffset;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");

        catapultMotor = hardwareMap.dcMotor.get("catapult");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        leftServo = hardwareMap.servo.get("leftServo");
        rightServo = hardwareMap.servo.get("rightServo");

        colorSensor = (AdafruitI2cColorSensor) hardwareMap.colorSensor.get("colorSensor");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        catapultMotor.setDirection(DcMotor.Direction.REVERSE);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        catapultMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotorOffset = backRightMotor.getCurrentPosition();
        catapultOffset = catapultMotor.getCurrentPosition();
        leftMotorOffset = backLeftMotor.getCurrentPosition();

        leftServo.scaleRange(0,1);
        rightServo.scaleRange(0,1);

    }

    @Override
    public void loop() {
        this.moveRobot(77-22, 0.3);

        this.activateCatapult();

        this.runIntake();

        this.activateCatapult();

        this.moveRobot(22, .3); //Temp auto stop here

        this.rotate(-90, 0.3); //TODO - Eventual Auto

        this.moveRobot(71, .3);

        //this.pressButton();

        requestOpModeStop();
    }

    @Override
    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private void moveRobot(int distance, double power) {
        while (backRightMotor.getCurrentPosition() - rightMotorOffset < targetPosition(distance)) {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(power);
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        this.resetEncoders();
    }

    /**
     * Method to convert inches to ticks
     * @param distance
     * @return distanceInTicks
     */
    private int targetPosition(double distance){
        double distanceInTicks = distance / INCHES_PER_TICK;
        return (int) distanceInTicks;
    }

    private void runUsingEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        catapultMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetEncoders() {
        leftMotorOffset = frontLeftMotor.getCurrentPosition();
        rightMotorOffset = frontRightMotor.getCurrentPosition();
        catapultOffset = catapultMotor.getCurrentPosition();
        intakeOffset = intakeMotor.getCurrentPosition();
    }

    private void activateCatapult(){
        while (catapultMotor.getCurrentPosition() - catapultOffset < ENCODER_TICKS_PER_REV * 3) {
            catapultMotor.setPower(1);
        }
        catapultMotor.setPower(0);

        catapultOffset = catapultMotor.getCurrentPosition();
    }

    private void runIntake(){
        while(intakeMotor.getCurrentPosition() + intakeOffset < INTAKE_TICKS){
            intakeMotor.setPower(0.4);
        }
        intakeMotor.setPower(0);
        this.resetEncoders();
    }

    private void rotate(int degrees, double power) {
        if(power < 0) throw new IllegalArgumentException("power = " + power);
        power = Range.clip(power, 0.0, 1.0);
        double leftPower = power;
        double rightPower = -power;
        if(degrees < 0) {
            leftPower = -power;
            rightPower = power;
            degrees = -degrees;
        }
        targetRotations = targetPosition(24 * Math.PI / (360 / degrees));
        while (backLeftMotor.getCurrentPosition() - leftMotorOffset < targetRotations) {
            frontLeftMotor.setPower(leftPower);
            frontRightMotor.setPower(rightPower);
            backLeftMotor.setPower(leftPower);
            backRightMotor.setPower(rightPower);
        }
    }
    private void buttonPress(){ //Blue
        if (colorSensor.red() > 1400 && colorSensor.blue() <1500 && colorSensor.green()< 1000){
            leftServo.setPosition(1);
        }
        else if(colorSensor.red() < 1000 && colorSensor.blue() >2000 && colorSensor.green()< 1000){
            rightServo.setPosition(1);
        }
    }

}
