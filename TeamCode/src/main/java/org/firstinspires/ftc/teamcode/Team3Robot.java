package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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
public class Team3Robot extends LinearOpMode {

    public enum FtcColor {
        RED,
        BLUE,
        NONE
    }

    /************** Constants **************/
    
    final static int ENCODER_TICKS_PER_REV = 1120; // Neverest 40
    final static int WHEEL_DIAMETER 	   = 4; // Inches
    final static double INCHES_PER_TICK    = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;
    
    final static double LEFT_SERVO_MIN     	 = 0.13;
    final static double LEFT_SERVO_HOME    	 = 0.74;
    final static double RIGHT_SERVO_MIN    	 = 0;
    final static double RIGHT_SERVO_HOME   	 = 0.55;
    final static double REGULATOR_SERVO_MIN  = 0;
    final static double REGULATOR_SERVO_HOME = 0.7;

    final static double HALF_SPEED = 0.5;
    
    final static int REGULATOR_TIME = 800;
    
    final static int DISTANCE_OFFSET      = 8;
    final static int SHOOTER_ROTATION 	  = 720;  // TODO - Find shooter ticks

    // TODO - Find offsets //
    //final static double LEFT_POWER_OFFSET  = 0.27;
    //final static double RIGHT_POWER_OFFSET = 0.40;
    final static int LED_PORT = 5;
    
    final static int LIFT_MINIMUM = 0;
    final static int LIFT_MAXIMUM = 500; // TODO - Find lift value
    
    final static double LIFT_POWER = 0.5;

    // Color sensor (TODO - Find thresholds) //
    final static int RED_THRESHOLD      = 495;
    final static int BLUE_THRESHOLD     = 1000;
    
    final static int GYRO_OFFSET = 17; // TODO - Is this needed?
    
    // Light sensor threshold //
    final static double LIGHT_THRESHOLD = 0.2;
    
    // Field constants //
    final static double DISTANCE_TO_VORTEX = 26 + DISTANCE_OFFSET;
    final static double DISTANCE_TO_BEACON = 4 + DISTANCE_OFFSET;
    final static int BEACON_DEGREES = 30;

    final static double LINE_POWER = 0.3;
    final static double BEACON_SPEED = -0.5;
    final static int WAIT_PERIOD = 250;
    
    // Member variables //
    protected int _leftOffset;
    protected int _rightOffset;
    
    protected int _intakeOffset;
    protected int _shooterOffset;
    
    protected int _liftOffset;

    protected int _gyroOffset;
    
    protected double _lightOffset;
    protected boolean _ledState;
    
    protected FtcColor _teamColor;

    // TeleOp Buttons //
    boolean _leftBeaconButton = false;
    boolean _rightBeaconButton = false;

    boolean _slowButton = false;

    // Hardware declarations //
    DcMotor _rightFrontMotor;
    DcMotor _leftFrontMotor;
    DcMotor _rightBackMotor;
    DcMotor _leftBackMotor;

    DcMotor _intakeMotor;
    DcMotor _shooterMotor;
    
    DcMotor _liftMotor;

    Servo _rightBeaconServo;
    Servo _leftBeaconServo;

    Servo _regulatorServo;

    ColorSensor _sensorRGB;
    DeviceInterfaceModule _cdim;

    ModernRoboticsI2cGyro _gyro;

    LightSensor _lightSensor;
    
    ElapsedTime _time = new ElapsedTime(Resolution.MILLISECONDS);

    /************** OpMode methods **************/

    private void baseInit() {
        _teamColor = FtcColor.NONE;
        _ledState = false;

        _leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        _leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        _rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        _rightBackMotor = hardwareMap.dcMotor.get("rightRear");

        _rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        _rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        _intakeMotor = hardwareMap.dcMotor.get("intake");
        _shooterMotor = hardwareMap.dcMotor.get("shooter");
        
        _shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        
        _liftMotor = hardwareMap.dcMotor.get("lift");

        _leftBeaconServo = hardwareMap.servo.get("leftBeacon");
        _rightBeaconServo = hardwareMap.servo.get("rightBeacon");

        _leftBeaconServo.scaleRange(LEFT_SERVO_MIN, LEFT_SERVO_HOME);
        _rightBeaconServo.scaleRange(RIGHT_SERVO_MIN, RIGHT_SERVO_HOME);
        
        _leftBeaconServo.setPosition(1);
        _rightBeaconServo.setPosition(1);
        
        _regulatorServo = hardwareMap.servo.get("regulator");

        _regulatorServo.scaleRange(REGULATOR_SERVO_MIN, REGULATOR_SERVO_HOME);

        _regulatorServo.setPosition(1);
        
        _sensorRGB = hardwareMap.colorSensor.get("color");

        _cdim = hardwareMap.deviceInterfaceModule.get("dim");
        _cdim.setDigitalChannelMode(LED_PORT, DigitalChannelController.Mode.OUTPUT);
        _cdim.setDigitalChannelState(LED_PORT, _ledState);

        _lightSensor = hardwareMap.lightSensor.get("light");

        _gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        _gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        
        this.calibrateGyro();
        this.resetGyro();
        
        this.runUsingEncoders();
        
        this.resetMotorEncoders();
        this.resetIntakeEncoder();
        this.resetShooterEncoder();
        
        _time.reset();
    }
    
    private void baseMain() { }

    private void baseStop() {
    	this.setPower(0);
    	this.setIntakePower(0);
    	this.setLiftPower(0);
    }
    
    protected void robotInit() { }
    
    protected void robotMain() { }
    
    protected void robotStop() { }
    
    @Override
    public void runOpMode() {
    	this.baseInit();
    	this.robotInit();
    	
    	this.waitForStart();
    	
    	this.baseMain();
    	this.robotMain();
    	
    	this.baseStop();
    	this.robotStop();
    }

    /************** Utility methods **************/

    private void runUsingEncoders() {
    	_leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        _intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        _liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private void runToPosition() {
    	_leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        _liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
    	//leftPower = this.scaleInput(leftPower);
    	//rightPower = this.scaleInput(rightPower);
    	
        _leftFrontMotor.setPower(leftPower);
        _leftBackMotor.setPower(leftPower);
        _rightFrontMotor.setPower(rightPower);
        _rightBackMotor.setPower(rightPower);
    }

    /**
     * Set shooter power
     * @param power
     */
    protected void setShooterPower(double power) {
        power = Range.clip(power, -1.0, 1.0);
        _shooterMotor.setPower(power);
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

        this.resetShooterEncoder();

        this.wait(Team3Robot.WAIT_PERIOD);
    }

    /**
     * Lifts regulator servo
     */
    protected void liftRegulator() {
        _regulatorServo.setPosition(0);
        this.wait(REGULATOR_TIME);
        _regulatorServo.setPosition(1);
    }

    protected void setIntakePower(double power) {
        power = Range.clip(power, -1.0, 1.0);
        _intakeMotor.setPower(power);
    }

    /**
     * Set the lift power
     * @param power
     */
    protected void setLiftPower(double power) {
    	power = Range.clip(power, -1.0, 1.0);

        if((this.getLiftPosition() <= 0 && power < 0) || (this.getLiftPosition() >= LIFT_MAXIMUM && power > 0)) {
            power = 0;
        }

        _liftMotor.setPower(power);
    }

    protected void setLeftServo(double position) {
        _leftBeaconServo.setPosition(position);
    }

    protected void setRightServo(double position) {
        _rightBeaconServo.setPosition(position);
    }

    protected double getLeftServo() {
        return _leftBeaconServo.getPosition();
    }

    protected double getRightServo() {
        return _rightBeaconServo.getPosition();
    }
    
    protected void setTeamColor(FtcColor teamColor) {
        _teamColor = teamColor;
    }

    protected int getLeftMotor() {
        return _leftFrontMotor.getCurrentPosition() - _leftOffset;
    }

    protected int getRightMotor() {
        return _rightFrontMotor.getCurrentPosition() - _rightOffset;
    }

    protected int getShooterPosition() {
        return _shooterMotor.getCurrentPosition() - _shooterOffset;
    }

    protected int getIntakePosition() {
        return _intakeMotor.getCurrentPosition() - _intakeOffset;
    }
    
    protected int getLiftPosition() {
    	return _liftMotor.getCurrentPosition() - _liftOffset;
    }

    protected int getGyroHeading() {
        return _gyro.getIntegratedZValue() - _gyroOffset;
    }

    protected double getLightReading() {
        return _lightSensor.getLightDetected() - _lightOffset;
    }
    
    protected boolean isLineDetected() {
        return Math.abs(this.getLightReading()) > LIGHT_THRESHOLD;
    }
    
    private double getTime() {
    	return _time.milliseconds();
    }

    private FtcColor getBeaconColor() {
        FtcColor beaconColor;
        // If red is less than threshold and blue is greater than red
        if(_sensorRGB.red() < RED_THRESHOLD && _sensorRGB.blue() > _sensorRGB.red()) {
            beaconColor = FtcColor.BLUE;
        } else {
            beaconColor = FtcColor.RED;
        }
        return beaconColor;
    }
    
    protected void resetMotorEncoders() {
        _leftOffset = _leftFrontMotor.getCurrentPosition();
        _rightOffset = _rightFrontMotor.getCurrentPosition();
    }
    
    protected void resetIntakeEncoder() {
    	_intakeOffset = _intakeMotor.getCurrentPosition();
    }
    
    protected void resetShooterEncoder() {
    	_shooterOffset = _shooterMotor.getCurrentPosition();
    }
    
    protected void resetLightSensor() {
        _lightOffset = _lightSensor.getLightDetected();
    }

    protected void resetBeaconServos() {
        _leftBeaconServo.setPosition(1);
        _rightBeaconServo.setPosition(1);
    }

    protected void resetGyro() {
        _gyroOffset = _gyro.getIntegratedZValue();
    }
    
    protected void calibrateGyro() {
    	_gyro.calibrate();
    	while(_gyro.isCalibrating()) {
    		telemetry.addData("gyro", "calibrating");
    		telemetry.update();
    	}
    	telemetry.addData("gyro", _gyro.getHeading());
    	telemetry.update();
    }

    protected boolean isGyroCalibrating() {
        return _gyro.isCalibrating();
    }

    protected void wait(int milliseconds) {
        int initTime = (int) this.getTime();
        while(this.getTime() - initTime < milliseconds) { }
    }

    /**
     * Runs the intake for a specified amount of time
     */
    protected void runIntake(int milliseconds) {
        _intakeMotor.setPower(1);
        this.wait(milliseconds);
        _intakeMotor.setPower(0);

        this.resetIntakeEncoder();

        this.wait(Team3Robot.WAIT_PERIOD);
    }

    private void driveToLine() {
    	_lightSensor.enableLed(true);
    	
        this.setPower(-LINE_POWER);
        
        while(!this.isLineDetected()) { }

        this.setPower(0);
        
        _lightSensor.enableLed(false);
    }

    private void alignToLine() {
        if(_teamColor == FtcColor.BLUE) {
            this.autoTurnInPlace(BEACON_DEGREES, LINE_POWER);
        } else {
            this.autoTurnInPlace(-BEACON_DEGREES, LINE_POWER);
        }
    }
    
    /**
     * Scale input according to a 16-value scaling array
     * @param dVal
     * @return dScale
     */
    private double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
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

        this.resetMotorEncoders();

        double distanceInTicks = distance / INCHES_PER_TICK;
        
        if(distance < 0) {
            while(this.getLeftMotor() > -distanceInTicks && this.getRightMotor() > -distanceInTicks) {
                this.setPower(-leftPower, -rightPower);
            }
        } else {
            while(this.getLeftMotor() < distanceInTicks && this.getRightMotor() < distanceInTicks) {
                this.setPower(leftPower, rightPower);
            }
        }
        this.setPower(0);

        this.wait(Team3Robot.WAIT_PERIOD);
    }

    /**
     * Method to turn degrees with specified power
     * @param degrees
     * @param power
     */
    protected void autoTurnInPlace(int degrees, double power) {
        if(power < 0) throw new IllegalArgumentException("power = " + power);

        this.resetGyro();
        
        if(degrees < -180) {
        	degrees = (degrees % 360) + 360;
        } else if(degrees > 180) {
        	degrees = (degrees % 360) - 360;
        }

        if(degrees < 0) {
        	this.setPower(-power, power);
            while(this.getGyroHeading() > degrees) {
                telemetry.addData("gyro", this.getGyroHeading());
                telemetry.update();
            }
        } else {
        	this.setPower(power, -power);
            while(this.getGyroHeading() < degrees) {
                telemetry.addData("gyro", this.getGyroHeading());
                telemetry.update();
            }
        }

        this.setPower(0, 0);
    }

    /**
     * Drives to the beacon and aligns
     */
    protected void autoDriveToBeacon() {
        this.driveToLine();
        this.alignToLine();
    }

    /**
     * Press the beacon with your team's color
     */
    protected void autoPressBeacon() {
        if(this.getBeaconColor() == _teamColor) {
            _leftBeaconServo.setPosition(LEFT_SERVO_MIN);
        } else {
            _rightBeaconServo.setPosition(RIGHT_SERVO_MIN);
        }
        this.wait(Team3Robot.WAIT_PERIOD);
        this.autoDriveDistance(DISTANCE_TO_BEACON, BEACON_SPEED);
        this.autoDriveDistance(-DISTANCE_TO_BEACON, BEACON_SPEED);
    }
}

