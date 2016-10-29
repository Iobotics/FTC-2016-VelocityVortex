package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name = "Team 2: Autonomous", group = "Team 2")
//@Disabled
public class Team2Auto extends OpMode {
	
	final int    ENCODER_TICKS_PER_REV = 1120; // Neverest 40
	final int    CHASSIS_DIAMETER	   = 18;
    final int    WHEEL_DIAMETER        = 10;
    final double CM_PER_TICK		   = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV; // CM / REV

    int catapultOffset;

    final int TARGET_POS = 3 * 1120; // Three rotations
	
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor intakeMotor;
    DcMotor catapultMotor;

    int leftMotorOffset;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");
        catapultMotor = hardwareMap.dcMotor.get("catapult");

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        catapultMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        long initialTime = System.currentTimeMillis();

        // Runs intake and shooter on start
    }

    @Override
    public void loop() {
        moveForward(127, 1.0);
        //activateCatapult();
        moveForward(23, 1.0);
    }
    
    /**
     * A method to calculate the number of ticks to move the robot a number of inches
     * @param distance
     * @return distanceInTicks
     */



    private int distance(double distance){
        double distanceInTicks = distance / CM_PER_TICK;
        return (int) distanceInTicks;
    }

    /**
     * A method to calculate amount of ticks to move the robot to rotate it
     * @param degrees
     * @return distanceInTicks
     */
    private int rotation(int degrees){
        double chassisCircumference = CHASSIS_DIAMETER * Math.PI;
        double centimetersPerDegree = chassisCircumference / 360;
        double distanceInTicks = centimetersPerDegree * degrees;
        return (int) distanceInTicks;
    }

    /**
     * Method that moves a target distance at a target power
     * @param targetDistance
     * @param targetPower
     */
    private void moveForward(int targetDistance, double targetPower){
        int currentPosition = 0;
        int distanceToTravel = distance(targetDistance);

        while ((frontLeftMotor.getCurrentPosition() - leftMotorOffset) > distanceToTravel){
            frontLeftMotor.setPower(targetPower);
            frontRightMotor.setPower(targetPower);
            backLeftMotor.setPower(targetPower);
            backRightMotor.setPower(targetPower);
            leftMotorOffset = frontLeftMotor.getCurrentPosition();
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private void activateCatapult(){
        while((catapultMotor.getCurrentPosition() - catapultOffset) < TARGET_POS) {
            catapultMotor.setPower(1);
        }
        catapultMotor.setPower(0);
        catapultOffset = catapultMotor.getCurrentPosition();
    }
}
