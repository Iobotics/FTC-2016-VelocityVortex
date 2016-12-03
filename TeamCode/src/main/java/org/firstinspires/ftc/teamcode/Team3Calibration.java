package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Team3Robot.FtcColor;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Darren Kam on 9/28/2016.
 *
 * Team 3 tank drive robot (Calibration)
 *
 * Controls:
 *
 * Left joystick - Left robot motors
 * Right joystick - Right robot motors
 * Left trigger - Shoots a ball
 * Left bumper - Manual shooter
 * Right trigger - Intake
 * Right bumper - Outtake
 * A - left beacon servo
 * B - Right beacon servo
 * X - N/A
 * Y - N/A
 * Start - Toggle half speed
 */
@TeleOp(name = "Team 3: Calibration", group = "Team 3")
//@Disabled
public class Team3Calibration extends OpMode {
	
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

    double speedMultiplier = 1;
    
    @Override
    public void init() {
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
    }
    
    @Override
    public void init_loop() {
    	telemetry.addData("Gyro", this.isGyroCalibrating() ? "Calibrating" : this.getGyroHeading());
    	telemetry.update();
    }

    @Override
    public void loop() {
        // Basic tank drive
    	this.setPower(gamepad1.left_stick_y * speedMultiplier, gamepad1.right_stick_y * speedMultiplier);

        // Left trigger to use shooter
        if(gamepad1.left_trigger > 0) {
        	this.shootBall();
            this.resetShooterEncoder();
        }
        else if(gamepad1.left_bumper) {
        	this.setShooterPower(0.6);
        }
        else {
        	this.setShooterPower(0);
        }

        // Activates intake when right trigger is pressed
        if(gamepad1.right_trigger > 0) {
        	this.setIntakePower(1);
        }
        else if(gamepad1.right_bumper) {
        	this.setIntakePower(-1);
        }
        else {
        	this.setIntakePower(0);
        }

        // Button A to toggle left beacon servo
        if(gamepad1.a && !_leftBeaconButton) {
        	this.setLeftServo((this.getLeftServo() < 0.2) ? 1 : 0);
            _leftBeaconButton = true;
        } else if(!gamepad1.a) _leftBeaconButton = false;

        // Button B to toggle right beacon servo
        if(gamepad1.b && !_rightBeaconButton) {
            this.setRightServo((this.getRightServo() < 0.2) ? 1 : 0);
            _rightBeaconButton = true;
        } else if(!gamepad1.b) _rightBeaconButton = false;

        if(gamepad1.x) {
            this.setLiftPower(Team3Robot.LIFT_POWER);
        }
        else if(gamepad1.y) {
        	this.setLiftPower(-Team3Robot.LIFT_POWER);
        }
        else {
        	this.setLiftPower(0);
        }

        // Start button to toggle half speed
        if(gamepad1.start && !_slowButton) {
            speedMultiplier = Team3Robot.HALF_SPEED;
            _slowButton = true;
        } else if(!gamepad1.start) _slowButton = false;

        telemetry.addData("Light Sensor", this.getLightReading());
        telemetry.addData("Line detected", (this.getLightReading() >= Team3Robot.LIGHT_THRESHOLD));
        telemetry.addData("Lift pos", this.getLiftPosition());
        telemetry.addData("Shooter pos", this.getShooterPosition());
        telemetry.addData("Gyro", this.getGyroHeading());
        telemetry.update();
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

    private FtcColor getBeaconColor() {
        FtcColor beaconColor = FtcColor.NONE;
        // If red is less than threshold and blue is greater than red
        if(_sensorRGB.red() > _sensorRGB.blue() && _sensorRGB.red() > _sensorRGB.green()) {
            beaconColor = FtcColor.RED;
        } else if(_sensorRGB.blue() > _sensorRGB.red() || _sensorRGB.green() > _sensorRGB.red()) {
            beaconColor = FtcColor.BLUE;
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
        long initTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - initTime < milliseconds) { }
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
}
