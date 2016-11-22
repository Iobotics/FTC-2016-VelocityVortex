package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Teacher on 9/28/2016.
 */

public class Team3Base extends LinearOpMode {

    protected enum FtcColor {
        RED,
        BLUE,
        NONE
    }

    // Robot constants //
    protected final int TICK_OFFSET 		  = 0;  // TODO - Calibrate
    protected final int SHOOTER_ROTATION 	  = 760;  // TODO - Calibrate
    protected final int WHEEL_DIAMETER 		  = 4;
    protected final int ENCODER_TICKS_PER_REV = 1120;
    protected final double LEFT_POWER_OFFSET  = 0.27; // TODO - Calibrate
    protected final double RIGHT_POWER_OFFSET = 0.40; // TODO - Calibrate
    protected final double LEFT_SERVO_MIN     = 0.132;
    protected final double RIGHT_SERVO_MIN    = 0;
    protected final double LEFT_SERVO_HOME    = 0.74;
    protected final double RIGHT_SERVO_HOME   = 0.55;
    protected final double REGULATOR_SERVO_MIN  = 0;
    protected final double REGULATOR_SERVO_HOME = 0.7;
    protected final double ROBOT_TURN_RADIUS  = 8; // Inches
    protected final double INCHES_PER_TICK 	  = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;
    // TODO - Find thresholds
    protected final int RED_THRESHOLD      = 3000;
    protected final int BLUE_THRESHOLD     = 3000;
    protected final double LIGHT_THRESHOLD = 0.24;

    // FTC Field constants //
    protected final double CENTER_TO_LINE = 3; // TODO - Calibrate
    protected final double BEACON_DISTANCE = 5; // TODO - Calibrate

    // Member variables //
    protected int leftOffset;
    protected int rightOffset;
    protected int intakeOffset;
    protected int shooterOffset;
    protected double lightOffset;
    protected boolean ledState;
    protected FtcColor teamColor;

    // Hardware declarations //
    private DcMotor rightFrontMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftBackMotor;

    private DcMotor intakeMotor;
    private DcMotor shooterMotor;

    private Servo rightBeaconServo;
    private Servo leftBeaconServo;

    private Servo regulatorServo;

    private ColorSensor sensorRGB;
    private DeviceInterfaceModule cdim;

    private ModernRoboticsI2cGyro gyro;

    protected LightSensor lightSensor;

    /************** OpMode methods **************/

    public void baseInit() {
        teamColor = FtcColor.NONE;

        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        rightBackMotor = hardwareMap.dcMotor.get("rightRear");

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        shooterMotor = hardwareMap.dcMotor.get("shooter");

        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        leftBeaconServo = hardwareMap.servo.get("leftBeacon");
        rightBeaconServo = hardwareMap.servo.get("rightBeacon");

        regulatorServo = hardwareMap.servo.get("regulator");

        leftBeaconServo.scaleRange(LEFT_SERVO_MIN, LEFT_SERVO_HOME);
        rightBeaconServo.scaleRange(RIGHT_SERVO_MIN, RIGHT_SERVO_HOME);

        regulatorServo.scaleRange(REGULATOR_SERVO_MIN, REGULATOR_SERVO_HOME);

        leftBeaconServo.setPosition(1);
        rightBeaconServo.setPosition(1);

        regulatorServo.setPosition(1);

        ledState = false;

        sensorRGB = hardwareMap.colorSensor.get("color");

        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        cdim.setDigitalChannelMode(5, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelState(5, ledState);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        this.calibrateGyro();

        lightSensor = hardwareMap.lightSensor.get("light");

        this.runUsingEncoders();
        this.resetEncoders();
    }

    public void baseMain() { }

    public void baseStop() { }

    public void runOpMode() {
        this.baseInit();
        this.robotInit();

        this.baseMain();
        this.robotMain();

        this.baseStop();
        this.robotStop();
    }

    protected void robotInit() { }

    protected void robotMain() { }

    protected void robotStop() { }

    /************** Utility methods **************/

    protected void runUsingEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void resetEncoders() {
        leftOffset = leftFrontMotor.getCurrentPosition();
        rightOffset = rightFrontMotor.getCurrentPosition();

        intakeOffset = intakeMotor.getCurrentPosition();
        shooterOffset = shooterMotor.getCurrentPosition();
    }

    protected void resetServos() {
        leftBeaconServo.setPosition(1);
        rightBeaconServo.setPosition(1);
    }

    protected void resetLightSensor() {
        lightOffset = lightSensor.getLightDetected();
    }

    protected void calibrateGyro() {
        gyro.calibrate();
        while(gyro.isCalibrating()) {
            this.setPower(0);
        }
    }

    /**
     * Set the power of the left and right motors
     * @param power
     */
    protected void setPower(double power) {
        this.setPower(power, power);
    }

    /**
     * Set the power of the left and right motors
     * @param leftPower
     * @param rightPower
     */
    protected void setPower(double leftPower, double rightPower) {
        leftFrontMotor.setPower(leftPower);
        leftBackMotor.setPower(leftPower);
        rightFrontMotor.setPower(rightPower);
        rightBackMotor.setPower(rightPower);
    }

    protected int getLeftPosition() {
        return leftFrontMotor.getCurrentPosition() - leftOffset;
    }

    protected int getRightPosition() {
        return rightFrontMotor.getCurrentPosition() - rightOffset;
    }

    protected int getShooterPosition() {
        return shooterMotor.getCurrentPosition() - shooterOffset;
    }

    protected int getIntakePosition() {
        return intakeMotor.getCurrentPosition() - intakeOffset;
    }

    protected double getLight() {
        return lightSensor.getLightDetected() - lightOffset;
    }

    protected void wait(int milliseconds) {
        long initTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - initTime < milliseconds) {
            this.setPower(0);
        }
    }

    protected void liftRegulator() {
        regulatorServo.setPosition(0);
        this.wait(800);
        regulatorServo.setPosition(1);
    }

    protected boolean lineDetected() {
        return (this.getLight() >= LIGHT_THRESHOLD);
    }

    // FIXME - Loop error
    protected void driveToLine() {
        while(!this.lineDetected()) {
            this.setPower(0.5);
        }
        this.setPower(0);
    }

    protected void alignToLine() {
        this.autoDriveDistance(CENTER_TO_LINE, 1.0);
        while(!this.lineDetected()) {
            if(teamColor == FtcColor.RED) {
                this.setPower(-1, 1);
            } else {
                this.setPower(1, -1);
            }
        }
        this.setPower(0);
    }


    /************** Auto commands **************/

    /**
     * Method to drive distance with positive power
     * @param distance
     * @param power (positive)
     */
    protected void autoDriveDistance(double distance, double power) {
        this.autoDriveDistance(distance, power, power);
    }

    /**
     * Method to drive distance with positive power
     * @param distance
     * @param leftPower (positive)
     * @param rightPower (positive)
     */
    protected void autoDriveDistance(double distance, double leftPower, double rightPower) {
        //if(leftPower < 0 || rightPower < 0) throw new IllegalArgumentException("power = " + ((leftPower < 0) ? leftPower : rightPower));
        if(distance < 0) {
            leftPower = -leftPower;
            rightPower = -rightPower;
            distance = -distance;
        }
        this.resetEncoders();

        // TODO - Offset for negative distance
        double distanceInTicks = (distance / INCHES_PER_TICK) + TICK_OFFSET;

        while(getLeftPosition() < distanceInTicks && getRightPosition() < distanceInTicks) {
            this.setPower(leftPower, rightPower);
        }
        this.setPower(0);

        this.wait(500);
    }

    /**
     * Method to drive distance with PID control
     * @param distance
     * @param power (positive)
     */
    // FIXME - Get this working
    private final double PID_DRIVE_GAIN             = 0.5;
    private final double PID_DRIVE_TOLERANCE_INCHES = 0.5;
    protected void autoDriveDistancePID(double distance) {
        if(distance < 0) {
            distance = -distance;
        }
        // FIXME - Offset for negative distance
        double distanceInTicks = (distance / INCHES_PER_TICK) + TICK_OFFSET;
        this.resetEncoders();
        double errorLeft = distanceInTicks - getLeftPosition();
        double errorRight = distanceInTicks - getRightPosition();
        double powerLeft;
        double powerRight;

        while(Math.abs(errorLeft) < PID_DRIVE_TOLERANCE_INCHES && Math.abs(errorRight) < PID_DRIVE_TOLERANCE_INCHES) {
            errorLeft = distanceInTicks - getLeftPosition();
            errorRight = distanceInTicks - getRightPosition();
            powerLeft = Range.clip(PID_DRIVE_GAIN * errorLeft, -1.0, 1.0);
            powerRight = Range.clip(PID_DRIVE_GAIN * errorRight, -1.0, 1.0);
            this.setPower(powerLeft, powerRight);
        }
        this.setPower(0, 0);

        this.resetEncoders();
    }

    /**
     * Method to turn degrees with positive power
     * @param degrees
     * @param power
     */
    // FIXME - Gets stuck in loop
    protected void autoTurnInPlace(int degrees, double power) {
        if(degrees < -360) throw new IllegalArgumentException("degrees = " + degrees);
        if(power < 0) throw new IllegalArgumentException("power = " + power);

        this.calibrateGyro();

        if(degrees < 0) {
            degrees += 360;
            while(gyro.getHeading() > degrees) {
                this.setPower(-power, power);
            }
        } else {
            while(gyro.getHeading() < degrees) {
                this.setPower(power, -power);
            }
        }
        this.setPower(0, 0);

        this.resetEncoders();
    }

    /**
     * Shoots a ball
     */
    protected void shootBall() {
        while(getShooterPosition() < SHOOTER_ROTATION) {
            shooterMotor.setPower(1);
        }
        shooterMotor.setPower(0);

        this.liftRegulator();

        this.resetEncoders();

        this.wait(250);
    }

    /**
     * Runs the intake with a specified amount of rotations
     */
    protected void runIntake() {
        while(getIntakePosition() < 4 * ENCODER_TICKS_PER_REV) {
            intakeMotor.setPower(1);
        }
        intakeMotor.setPower(0);

        this.resetEncoders();

        this.wait(250);
    }

    protected void autoDriveToBeacon() {
        this.driveToLine();
        //this.alignToLine();
        //this.autoDriveDistance(BEACON_DISTANCE, 1.0);
    }

    /**
     * Press the beacon with your team's color
     */
    protected void autoPressBeacon() {
        if(sensorRGB.red() >= RED_THRESHOLD && sensorRGB.blue() <= BLUE_THRESHOLD && teamColor == FtcColor.RED) {
            leftBeaconServo.setPosition(LEFT_SERVO_MIN);
        } else {
            rightBeaconServo.setPosition(RIGHT_SERVO_MIN);
        }
    }
}

