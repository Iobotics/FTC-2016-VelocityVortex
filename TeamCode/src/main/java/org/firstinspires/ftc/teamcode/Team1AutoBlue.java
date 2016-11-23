package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name = "Team 1 Blue: Autonomous", group = "Team 1 Blue")
//@Disabled
public class Team1AutoBlue extends LinearOpMode {

	final int    ENCODER_TICKS_PER_REV = 1120; // Neverest 40
    final int    WHEEL_DIAMETER        = 6;
    final double INCHES_PER_TICK       = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV; // INCHES / REV
    final int INTAKE_TICKS             = ENCODER_TICKS_PER_REV * 1;

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    DcMotor catapultMotor;
    DcMotor intakeMotor;

    int catapultOffset;
    int rightMotorOffset;
    int leftMotorOffset;

    int targetRotations;
    int intakeOffset;

    public void robotInit() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");

        catapultMotor = hardwareMap.dcMotor.get("catapult");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        this.runUsingEncoders();

        this.resetEncoders();
    }

    public void robotMain() {

        this.moveRobot(45, 0.3);

        //this.activateCatapult();

        //this.runIntake();

        //this.activateCatapult();

        //this.rotate(90, 0.3); //TODO - Eventual Auto

        //this.moveRobot(60, .3);

        //this.pressButton();

        //this.move Robot(60, .3);
    }

    public void robotStop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    @Override
    public void runOpMode() {
        this.robotInit();

        this.robotMain();

        this.robotStop();
    }

    private void moveRobot(int distance, double power) {
        this.resetEncoders();

        int distanceInTicks = (int) (distance / INCHES_PER_TICK);
        while (frontLeftMotor.getCurrentPosition() - leftMotorOffset < distanceInTicks) {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(power);
        }

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
        leftMotorOffset = frontLeftMotor.getCurrentPosition();
        rightMotorOffset = -frontRightMotor.getCurrentPosition();
        catapultOffset = catapultMotor.getCurrentPosition();
        intakeOffset = -intakeMotor.getCurrentPosition();
    }

    private void activateCatapult(){
        while ((-catapultMotor.getCurrentPosition()) - catapultOffset < ENCODER_TICKS_PER_REV * 3) {
            catapultMotor.setPower(1);
        }
        catapultMotor.setPower(0);

        catapultOffset = -catapultMotor.getCurrentPosition();
    }

    private void runIntake(){
        while((-intakeMotor.getCurrentPosition()) - intakeOffset < INTAKE_TICKS){
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
        while (frontLeftMotor.getCurrentPosition() - leftMotorOffset < targetRotations) {
                frontLeftMotor.setPower(leftPower);
                frontRightMotor.setPower(rightPower);
                backLeftMotor.setPower(leftPower);
                backRightMotor.setPower(rightPower);
        }
    }
    /*
    private void buttonPress(){ //Blue
        if (colorSensor.red() > 1400 && colorSensor.blue() <1500 && colorSensor.green()< 1000){

        }
        else if(colorSensor.red() < 1000 && colorSensor.blue() >2000 && colorSensor.green()< 1000){

        }
    }
    */
}
