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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime.Resolution;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Darren Kam on 9/28/2016.
 */
public class Team3Base extends LinearOpMode {

    protected enum FtcColor {
        RED,
        BLUE,
        NONE
    }

    /************** Constants **************/
    
    final static int ENCODER_TICKS_PER_REV = 1120; // Neverest 40
    final static int WHEEL_DIAMETER 	   = 4; // Inches
    final static double INCHES_PER_TICK    = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;
    
    final static double LEFT_SERVO_MIN     = 0.13;
    final static double LEFT_SERVO_HOME    = 0.74;
    final static double RIGHT_SERVO_MIN    = 0;
    final static double RIGHT_SERVO_HOME   = 0.55;
    final static double REGULATOR_SERVO_MIN  = 0;
    final static double REGULATOR_SERVO_HOME = 0.7;
    
    final static double REGULATOR_TIME = 800;
    
    final static int TICK_OFFSET 		  = 0;  // TODO - Find ticks to offset distance
    final static int SHOOTER_ROTATION 	  = 760;  // TODO - Find shooter ticks
    
    final static double LEFT_POWER_OFFSET  = 0.27;
    final static double RIGHT_POWER_OFFSET = 0.40;
    
    // Color sensor (TODO - Find thresholds) //
    final static int RED_THRESHOLD      = 3000;
    final static int BLUE_THRESHOLD     = 3000;
    
    // Light sensor threshold //
    final static double LIGHT_THRESHOLD = 0.24;

    // Field constants //
    final static double CENTER_TO_LINE = 3;
    final static double DISTANCE_TO_BEACON = 5;
    
    // Member variables //
    protected int _leftOffset;
    protected int _rightOffset;
    
    protected int _intakeOffset;
    protected int _shooterOffset;
    
    protected double _lightOffset;
    protected boolean _ledState;
    
    protected FtcColor _teamColor;

    // Hardware declarations //
    DcMotor _rightFrontMotor;
    DcMotor _leftFrontMotor;
    DcMotor _rightBackMotor;
    DcMotor _leftBackMotor;

    DcMotor _intakeMotor;
    DcMotor _shooterMotor;
    //DcMotor _liftMotor;

    Servo _rightBeaconServo;
    Servo _leftBeaconServo;

    Servo _regulatorServo;

    ColorSensor _sensorRGB;
    DeviceInterfaceModule _cdim;

    ModernRoboticsI2cGyro _gyro;

    LightSensor _lightSensor;
    
    ElapsedTime _time = new ElapsedTime(Resolution.MILLISECONDS);

    /************** OpMode methods **************/

    public void baseInit() {
    	_teamColor = FtcColor.NONE;

        _leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        _leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        _rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        _rightBackMotor = hardwareMap.dcMotor.get("rightRear");

        _leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        _leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        _intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        _shooterMotor = hardwareMap.dcMotor.get("shooter");
        //_liftMotor = hardwareMap.dcMotor.get("lift");

        _shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        _leftBeaconServo = hardwareMap.servo.get("leftBeacon");
        _rightBeaconServo = hardwareMap.servo.get("rightBeacon");

        _regulatorServo = hardwareMap.servo.get("regulator");

        _leftBeaconServo.scaleRange(LEFT_SERVO_MIN, LEFT_SERVO_HOME);
        _rightBeaconServo.scaleRange(RIGHT_SERVO_MIN, RIGHT_SERVO_HOME);

        _regulatorServo.scaleRange(REGULATOR_SERVO_MIN, REGULATOR_SERVO_HOME);

        _leftBeaconServo.setPosition(1);
        _rightBeaconServo.setPosition(1);

        _regulatorServo.setPosition(1);

        _ledState = false;
        
        _sensorRGB = hardwareMap.colorSensor.get("color");

        _cdim = hardwareMap.deviceInterfaceModule.get("dim");
        _cdim.setDigitalChannelMode(5, DigitalChannelController.Mode.OUTPUT);
        _cdim.setDigitalChannelState(5, _ledState);

        _lightSensor = hardwareMap.lightSensor.get("light");

        _gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        _gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        
        this.calibrateGyro();
        
        this.runUsingEncoders();
        
        this.resetMotors();
        this.resetIntake();
        this.resetShooter();
        
        _time.reset();
    }

    public void baseMain() { }

    public void baseStop() {
    	this.setPower(0);
    }

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
    	_leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        _intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //_liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void resetMotors() {
        _leftOffset = _leftFrontMotor.getCurrentPosition();
        _rightOffset = _rightFrontMotor.getCurrentPosition();
    }
    
    protected void resetIntake() {
    	_intakeOffset = _intakeMotor.getCurrentPosition();
    }
    
    protected void resetShooter() {
    	_shooterOffset = _shooterMotor.getCurrentPosition();
    }
    
    protected void resetLightSensor() {
        _lightOffset = _lightSensor.getLightDetected();
    }

    protected void resetBeaconServos() {
        _leftBeaconServo.setPosition(1);
        _rightBeaconServo.setPosition(1);
    }

    protected void calibrateGyro() {
        _gyro.calibrate();
        while(_gyro.isCalibrating()) {
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
        _leftFrontMotor.setPower(leftPower);
        _leftBackMotor.setPower(leftPower);
        _rightFrontMotor.setPower(rightPower);
        _rightBackMotor.setPower(rightPower);
    }

    protected int getLeftPosition() {
        return _leftFrontMotor.getCurrentPosition() - _leftOffset;
    }

    protected int getRightPosition() {
        return _rightFrontMotor.getCurrentPosition() - _rightOffset;
    }

    protected int getShooterPosition() {
        return _shooterMotor.getCurrentPosition() - _shooterOffset;
    }

    protected int getIntakePosition() {
        return _intakeMotor.getCurrentPosition() - _intakeOffset;
    }

    protected double getLight() {
        return _lightSensor.getLightDetected() - _lightOffset;
    }
    
    protected double getTime() {
    	return _time.time();
    }

    protected void wait(int milliseconds) {
        int initTime = (int) _time.milliseconds();
        while(_time.milliseconds() - initTime < milliseconds) {
        	this.setPower(0.0);
        }
    }

    protected void liftRegulator() {
        _regulatorServo.setPosition(0);
        this.wait(800);
        _regulatorServo.setPosition(1);
    }

    protected boolean lineDetected() {
        return Math.abs(this.getLight()) > LIGHT_THRESHOLD;
    }

    protected void driveToLine() {
        while(!this.lineDetected()) {
            this.setPower(0.5);
        }
        this.setPower(0);
    }

    protected void alignToLine() {
        while(!this.lineDetected()) {
            if(_teamColor == FtcColor.RED) {
                this.setPower(-1.0, 1.0);
            } else {
                this.setPower(1.0, -1.0);
            }
        }
        this.setPower(0.0);
    }


    /************** Auto commands **************/

    /**
     * Method to drive distance with positive power
     * @param distance
     * @param power
     */
    protected void autoDriveDistance(double distance, double power) {
        this.autoDriveDistance(distance, power, power);
    }

    /**
     * Method to drive distance with positive power
     * @param distance
     * @param leftPower
     * @param rightPower
     */
    protected void autoDriveDistance(double distance, double leftPower, double rightPower) {
        if(leftPower < 0 || rightPower < 0) throw new IllegalArgumentException("left power = " + leftPower + "right power = " + rightPower);
        
        if(distance < 0) {
            leftPower = -leftPower;
            rightPower = -rightPower;
            distance = -distance;
        }
        
        this.resetMotors();

        double distanceInTicks = distance / INCHES_PER_TICK;

        while(getLeftPosition() < distanceInTicks && getRightPosition() < distanceInTicks) {
            this.setPower(leftPower, rightPower);
        }
        this.setPower(0.0);

        this.wait(500);
    }

    /**
     * Method to drive distance with PID control
     * @param distance
     * @param power
     */
    // FIXME - Get this working
    /*private final double PID_DRIVE_GAIN             = 0.5; // TODO - Find value
    private final double PID_DRIVE_TOLERANCE_INCHES = 0.5; // TODO - Find value
    protected void autoDriveDistancePID(double distance) {
    	
        double distanceInTicks = distance / INCHES_PER_TICK;
        
        this.resetMotors();
        
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
        this.setPower(0.0);

        this.resetMotors();
    }*/

    /**
     * Method to turn degrees with positive power
     * @param degrees
     * @param power
     */
    // FIXME - Doesn't move
    protected void autoTurnInPlace(int degrees, double power) {
        if(degrees < -360) throw new IllegalArgumentException("degrees = " + degrees);
        if(power < 0) throw new IllegalArgumentException("power = " + power);

        this.calibrateGyro();

        if(degrees < 0) {
            degrees += 360;
            while(_gyro.getHeading() > degrees) {
                this.setPower(-power, power);
            }
        } else {
            while(_gyro.getHeading() < degrees) {
                this.setPower(power, -power);
            }
        }
        this.setPower(0, 0);
    }

    /**
     * Shoots a ball and reloads from the regulator servo
     */
    protected void shootBall() {
        while(getShooterPosition() < SHOOTER_ROTATION) {
            _shooterMotor.setPower(1);
        }
        _shooterMotor.setPower(0);

        this.liftRegulator();

        this.resetShooter();

        this.wait(250);
    }

    /**
     * Runs the intake with a specified amount of rotations
     */
    protected void runIntake(int rotations) {
        while(getIntakePosition() < rotations * ENCODER_TICKS_PER_REV) {
            _intakeMotor.setPower(1);
        }
        _intakeMotor.setPower(0);

        this.resetIntake();

        this.wait(250);
    }

    /**
     * Drives to the beacon
     */
    protected void autoDriveToBeacon() {
        this.driveToLine();
        this.autoDriveDistance(CENTER_TO_LINE, 1.0);
        this.alignToLine();
        this.autoDriveDistance(DISTANCE_TO_BEACON, 1.0);
    }

    /**
     * Press the beacon with your team's color
     */
    protected void autoPressBeacon() {
        if(_sensorRGB.red() >= RED_THRESHOLD && _sensorRGB.blue() <= BLUE_THRESHOLD && _teamColor == FtcColor.RED) {
            _leftBeaconServo.setPosition(LEFT_SERVO_MIN);
        } else {
            _rightBeaconServo.setPosition(RIGHT_SERVO_MIN);
        }
    }
}

