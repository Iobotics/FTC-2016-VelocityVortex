package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.ar.pl.SystemTools;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name = "Team 1: Autonomous w/o encoders", group = "Team 1")
@Disabled
public class Team1AutoWithoutEncoders extends OpMode {

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

    int leftOffset;
    int catapultOffset;

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

        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        catapultMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    }

    @Override
    public void loop() {
        moveForwardTime(1800, 1.0);

        activateCatapultTime();

        activateIntake(1500, 1);

        activateCatapultTime();

        moveForwardTime(900, 1.0);

        this.requestOpModeStop();
    }

    @Override

    public void stop(){
        time.reset();
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

    /**
     * Method that moves a target distance at a target power
     * @param targetDistance
     * @param targetPower
     */
    private void moveForward(int targetDistance, double targetPower){
        int distanceToTravel = distance(targetDistance);

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

        leftOffset = frontLeftMotor.getCurrentPosition();
    }
    /*
    Method that turns a trget amount of degrees at target power in target direction
    */
    private void turnDegrees (int targetDegrees, double targetPower, char direction) {
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
    }

    private void activateCatapultTime() {
        time.reset();

        while(time.milliseconds() < 1700) {
            catapultMotor.setPower(1);
        }

        catapultMotor.setPower(0);
    }

    private void moveForwardTime(double targetTimeMil, double targetPower){
        time.reset();

        while(time.milliseconds() < targetTimeMil){
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

    private void activateIntake(double targetTimeMil, int targetPowerIntake){
        time.reset();

        while(time.milliseconds() < targetTimeMil) {
            intakeMotor.setPower(targetPowerIntake);
        }
        intakeMotor.setPower(0);
    }

}
