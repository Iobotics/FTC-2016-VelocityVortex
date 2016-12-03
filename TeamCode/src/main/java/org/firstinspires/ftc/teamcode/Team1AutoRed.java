/* This Autonomous goes forward, shoots two balls,
 * goes 90 degrees, hits the beacon, and parks on the center vortex.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name = "Team 1 Red: Autonomous", group = "Team 1 Red")
//@Disabled
public class Team1AutoRed extends LinearOpMode {

    final int    ENCODER_TICKS_PER_REV = 1120; // Neverest 40
    final int    WHEEL_DIAMETER        = 6;
    final double INCHES_PER_TICK       = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV; // INCHES / REV
    final int INTAKE_TICKS             = ENCODER_TICKS_PER_REV * 5;

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    DcMotor catapultMotor;
    DcMotor intakeMotor;

    ModernRoboticsI2cGyro gyro;

    int catapultOffset;
    int rightMotorOffset;
    int leftMotorOffset;
    int gyroHeadingOffset;

    int targetRotations;
    int intakeOffset;

    public void robotInit() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");

        catapultMotor = hardwareMap.dcMotor.get("catapult");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        //gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        //frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        //frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        //intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        this.runUsingEncoders();

        this.resetEncoders();

        // gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        // gyro.calibrate();
    }

    public void robotMain() {

        this.moveRobot(50, 0.3);

        this.activateCatapult(); //Shoots ball

        this.runIntake();

        this.activateCatapult(); // Shoots another ball

        this.moveRobot(50, .3); //Right next to cat ball

        this.ballHit();

        this.moveRobot(40, .3);

        /*
        this.rotate(-35, .3);

        this.moveRobot(66, -.3);

        this.pressButton(); // button press blue

        this.move Robot(50, -.3); // 50 in backwards
    */
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

        waitForStart();

        this.robotMain();

        this.robotStop();
    }

    /**
     *  Method finds frontLeft Motor Encoder Position
     *  @return LeftEncoderPosition
     */
    private int getLeftPosition() {return (frontLeftMotor.getCurrentPosition() - leftMotorOffset); }

    /**
     *  Method finds intake Motor Encoder Position
     *  Encoder Value returns negative
     *  @return IntakeEncoderPositionF
     */
    private int getIntakePosition() {return (-intakeMotor.getCurrentPosition()) - intakeOffset; }

    /**
     *  Method finds Catapult Motor Encoder Position
     *  @return CatapultEncoderPosition
     */
    private int getCatapultPosition() {return (catapultMotor.getCurrentPosition() - catapultOffset); }

    /**
     * Moves robot in a line
     * @param distance
     * @param power
     * @return none
     */
    private void moveRobot(int distance, double power) {
        this.resetEncoders();

        int distanceInTicks = (int) (distance / INCHES_PER_TICK);
        while (getLeftPosition() < distanceInTicks) {
            frontLeftMotor.setPower(-power);
            frontRightMotor.setPower(-power);
            backLeftMotor.setPower(-power);
            backRightMotor.setPower(-power);
            telemetry.addData("Left position", getLeftPosition());
            telemetry.update();
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

    /**
     * Activates Encoders
     */
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

    /**
     * Shoots Catapult
     */
    private void activateCatapult(){
        while (getCatapultPosition() < ENCODER_TICKS_PER_REV * 3) {
            catapultMotor.setPower(1);
        }
        catapultMotor.setPower(0);

        catapultOffset = catapultMotor.getCurrentPosition();
    }

    private void runIntake(){
        this.resetEncoders();

        while(getIntakePosition() < INTAKE_TICKS){
            intakeMotor.setPower(-1);
        }
        intakeMotor.setPower(0);
    }

    /**
     * Rotates using gyro sensor
     * @param degrees
     * @param power
     */
    private void rotate(int degrees, double power) {
        if(power < 0) throw new IllegalArgumentException("power = " + power);

        this.resetGyroHeading();

        while (this.getGyroHeading() < degrees) {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(-power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(-power);
        }
    }


    private  void ballHit(){
        this.resetEncoders();

        while (getLeftPosition() < 3100){
            frontLeftMotor.setPower(-.3);
            backLeftMotor.setPower(-.3);
        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);

        this.resetEncoders();

        while (getLeftPosition() > -3100) {
            frontLeftMotor.setPower(.3);
            backLeftMotor.setPower(.3);
        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
    }

    protected void calibrateGyro() {
        gyro.calibrate();
    }

    protected boolean isGyroCalibrating() {
        return gyro.isCalibrating();
    }

    protected int getGyroHeading() { return gyro.getIntegratedZValue() - gyroHeadingOffset; }

    protected void resetGyroHeading() {
        gyroHeadingOffset = gyro.getIntegratedZValue();
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
