package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    final int INTAKE_MOVEMENT = ENCODER_TICKS_PER_REV * 1;
	
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    
    DcMotor catapultMotor;
    DcMotor intakeMotor;

    int leftOffset;
    int rightOffset;
    int catapultOffset;
    int intakeOffset;
    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");
        
        catapultMotor = hardwareMap.dcMotor.get("catapult");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        this.runUsingEncoders();
        this.resetEncoders();
        //frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        
        catapultMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        drive(55-22,.3);
        shootCatapult();
        runIntake();
        shootCatapult();
        requestOpModeStop();
    }

    @Override
    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
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
        leftOffset = frontLeftMotor.getCurrentPosition();
        rightOffset = frontRightMotor.getCurrentPosition();
        catapultOffset = catapultMotor.getCurrentPosition();
        intakeOffset = intakeMotor.getCurrentPosition();
    }

    private void setPower(double left, double right) {
        frontLeftMotor.setPower(left);
        frontRightMotor.setPower(right);
        backLeftMotor.setPower(left);
        backRightMotor.setPower(right);
    }

    private void shootCatapult() {
        while(catapultMotor.getCurrentPosition() + catapultOffset < CATAPULT_ROTATIONS) {
            catapultMotor.setPower(0.3);
        }
        catapultMotor.setPower(0);
        this.resetEncoders();
    }

    private void runIntake(){
        while(intakeMotor.getCurrentPosition() + intakeOffset < INTAKE_MOVEMENT){
            intakeMotor.setPower(0.4);
        }
        intakeMotor.setPower(0);
        this.resetEncoders();
    }

    /**
     * Method to drive distance with positive power
     * @param distance
     * @param power (positive)
     */
    private void drive(double distance, double power) {
        telemetry.addData("Target pos", (distance / INCHES_PER_TICK) + TICK_OFFSET);
        telemetry.update();
        if(power < 0) throw new IllegalArgumentException("power = " + power);
        if(distance < 0) {
            power = -power;
            distance = -distance;
        }
        this.resetEncoders();
        while(frontLeftMotor.getCurrentPosition() - leftOffset < (distance / INCHES_PER_TICK) + TICK_OFFSET) {
            this.setPower(power, power);
            telemetry.addData("Current pos", frontLeftMotor.getCurrentPosition());
            telemetry.update();
        }
        this.setPower(0, 0);
        this.resetEncoders();
    }

    private void turnDrive(double distance, double leftPower, double rightPower) {
        telemetry.addData("Target pos", (distance / INCHES_PER_TICK) + TICK_OFFSET);
        telemetry.update();
        if(leftPower < 0) throw new IllegalArgumentException("power = " + leftPower);
        if(rightPower < 0) throw new IllegalArgumentException("power = " + leftPower);
        if(distance < 0) {
            leftPower = -leftPower;
            rightPower = -rightPower;
            distance = -distance;
        }
        this.resetEncoders();
        while(frontLeftMotor.getCurrentPosition() - leftOffset < (distance / INCHES_PER_TICK) + TICK_OFFSET) {
            this.setPower(leftPower, rightPower);
            telemetry.addData("Current pos", frontLeftMotor.getCurrentPosition());
            telemetry.update();
        }
        this.setPower(0, 0);
        this.resetEncoders();
    }

    private void tempAuto(){
        drive(55-22,.3);
        shootCatapult();
        runIntake();
        shootCatapult();
        drive(22,.3);
    }

    private void eventualAuto(){
        drive(55-22,.3);
        shootCatapult();
        runIntake();
        shootCatapult();
    }
}
