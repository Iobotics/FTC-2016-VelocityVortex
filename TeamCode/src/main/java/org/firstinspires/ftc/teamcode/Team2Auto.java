package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.ar.pl.SystemTools;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name = "Team 2: Autonomous", group = "Team 2")
//@Disabled
public class Team2Auto extends OpMode {
	final int    ENCODER_TICKS_PER_REV = 1120; // Neverest 40
	final int 	 CATAPULT_TICKS 	   = 3 * ENCODER_TICKS_PER_REV; // Three rotations
	final int    CHASSIS_DIAMETER	   = 18; // CM
    final int    WHEEL_DIAMETER        = 10; // CM
    final double CM_PER_TICK		   = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV; // CM / REV


    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor intakeMotor;
    DcMotor catapultMotor;
    ModernRoboticsI2cGyro gyro;

    int leftOffset;
    int catapultOffset;
    int intakeOffset;

    long startTime;

    ElapsedTime time;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");

        catapultMotor = hardwareMap.dcMotor.get("catapult");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        gyro =  (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        useEncoders();

        time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        gyro.calibrate();
    }

    @Override
    public void loop() {

        moveForward(25,1);

        activateCatapult();

        activateIntakeTime(2000,1);

        activateCatapult();

        moveForward(25,1);

        this.requestOpModeStop();
    }

    @Override

    public void stop() {
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }
//*****************************************************************Utility*Methods***********************************************************
    private void useEncoders(){
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        catapultMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /**
     * A method to calculate the number of ticks to move the robot a number of inches
     * @param distance
     * @return distanceInTicks
     */private int distance(double distance){
        double distanceInTicks = distance / CM_PER_TICK;
        return (int) distanceInTicks;
    }

    /**
     * A method to calculate amount of ticks to rotate a specified number of degrees
     * @param degrees
     * @return distanceInTicks
     */
    private int rotation(int degrees){
        double chassisCircumference = CHASSIS_DIAMETER * Math.PI;
        double centimetersPerDegree = chassisCircumference / 360;
        double distanceInCm = centimetersPerDegree * degrees;
        double distanceInTicks = distanceInCm / CM_PER_TICK;
        return (int) distanceInTicks;
    }
    //*****************************************************OPMode*Methods****************************************************

    /**
     * Method that moves the robot forward a specified number of inches.
     * @param targetDistance
     * @param targetPower
     */
    private void moveForward(int targetDistance, double targetPower){
        int distanceToTravel = distance(targetDistance);
        leftOffset = frontLeftMotor.getCurrentPosition();



        while((frontLeftMotor.getCurrentPosition() - leftOffset) < distanceToTravel) {
            frontLeftMotor.setPower(targetPower);
            frontRightMotor.setPower(targetPower);
            backLeftMotor.setPower(targetPower);
            backRightMotor.setPower(targetPower);
        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

    }
    /*
    Method that turns a ta rget amount of degrees at target power in target direction
    */
    private void turnDegreesUsingEncoders (int targetDegrees, double targetPower, char direction) {
        int distanceToTurn = distance(rotation(targetDegrees));

        if (direction == 'L') {
            while ((frontLeftMotor.getCurrentPosition() - leftOffset) < distanceToTurn) {
                frontLeftMotor.setPower(targetPower);
                backLeftMotor.setPower(targetPower);
                frontRightMotor.setPower(-targetPower);
                backRightMotor.setPower(-targetPower);
            }
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }

        if (direction == 'R') {
            while ((frontLeftMotor.getCurrentPosition() - leftOffset) < distanceToTurn) {
                frontLeftMotor.setPower(-targetPower);
                backLeftMotor.setPower(-targetPower);
                frontRightMotor.setPower(targetPower);
                backRightMotor.setPower(targetPower);
            }
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }

    /**
     * Method to fire the catapult
     */
    private void activateCatapult() {
        while((catapultMotor.getCurrentPosition() - catapultOffset) < CATAPULT_TICKS) {
            catapultMotor.setPower(1);
        }
        catapultMotor.setPower(0);
        catapultOffset = catapultMotor.getCurrentPosition();
    }

    private void activateCatapultTime() {
        startTime = System.currentTimeMillis();

        while((System.currentTimeMillis() - startTime) < 1200) {
            catapultMotor.setPower(1);
        }

        catapultMotor.setPower(0);
    }

    /**
     * Moves forward for a target duration using system time
     * @param targetTimeMil
     * @param targetPower
     */
    private void moveForwardTime(long targetTimeMil, double targetPower){
        startTime = System.currentTimeMillis();

        while((System.currentTimeMillis() - startTime) < targetTimeMil){
            frontLeftMotor.setPower(targetPower);
            backLeftMotor.setPower(targetPower);
            backRightMotor.setPower(targetPower);
            frontRightMotor.setPower(targetPower);
        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    /**
     * rotates the intake by an amount of target rotations using encoders
     * @param targetRotations
     * @param targetPower
     */
    private void activateIntake(int targetRotations, double targetPower){
        intakeOffset = intakeMotor.getCurrentPosition();
        while((intakeMotor.getCurrentPosition() - intakeOffset) < targetRotations * 1120){
            intakeMotor.setPower(targetPower);
        }
        intakeMotor.setPower(0);
    }

    /**
     * Runs the intake for a chosen duration at target power using system time
     * @param targetDuration
     * @param targetPower
     */
    private void activateIntakeTime(int targetDuration, double targetPower){
        startTime = System.currentTimeMillis();

        while((System.currentTimeMillis() - startTime) < targetDuration){
            intakeMotor.setPower(targetPower);
        }
        intakeMotor.setPower(0);
    }

    /**
     * Method that turns target degrees using the gyro sensor
     * @param targetPower
     * @param targetDegrees
     */
    private void turnUsingGyro(double targetPower, int targetDegrees){
        gyro.calibrate();
        if(targetDegrees > 180){
            targetPower = -targetPower;
        }
        while(gyro.getHeading() < targetDegrees){
            frontLeftMotor.setPower(targetPower);
            frontRightMotor.setPower(-targetPower);
            backLeftMotor.setPower(targetPower);
            backRightMotor.setPower(-targetPower);
        }
        gyro.calibrate();
    }
}
