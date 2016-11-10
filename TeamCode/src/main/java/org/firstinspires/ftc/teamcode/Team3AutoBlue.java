package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.ams.AMSColorSensorImpl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name = "Team 3: AutonomousBlue", group = "Team 3")
//@Disabled
public class Team3AutoBlue extends OpMode {
	
	public enum FtcColor {
        RED,
        BLUE,
        NONE
    }

	// Constants //
    final int TICK_OFFSET 			= 450;  // TODO - Calibrate
    final int SHOOTER_ROTATION 		= 765;  // TODO - Calibrate
    final int WHEEL_DIAMETER 		= 4;
    final int ENCODER_TICKS_PER_REV = 1120;
    final double LEFT_POWER_OFFSET 	= 0.27; // TODO - Calibrate
    final double RIGHT_POWER_OFFSET = 0; // TODO - Calibrate
    final double LEFT_SERVO_MIN 	= 0.132;
    final double RIGHT_SERVO_MIN	= 0;
    final double LEFT_SERVO_HOME 	= 0.74;
    final double RIGHT_SERVO_HOME 	= 0.55;
    final double ROBOT_RADIUS 		= 8; // Inches
    final double INCHES_PER_TICK 	= (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;
    
    // TODO - Find thresholds
    final int RED_THRESHOLD  = 3000;
    final int BLUE_THRESHOLD = 3000;

    // Member variables //
    int leftOffset;
    int rightOffset;
    int shooterOffset;
    
    FtcColor teamColor;
    
    ElapsedTime time;

    // Hardware declarations //
    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor leftBackMotor;
    
    DcMotor intakeMotor;
    DcMotor shooterMotor;
    
    Servo rightBeaconServo;
    Servo leftBeaconServo;

    AMSColorSensorImpl sensorRGB;

    ModernRoboticsI2cGyro gyro;
    
    @Override
    public void init() {
        teamColor = FtcColor.BLUE;
        time = new ElapsedTime();

        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        rightBackMotor = hardwareMap.dcMotor.get("rightRear");
        
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        shooterMotor = hardwareMap.dcMotor.get("shooter");
        
        leftBeaconServo = hardwareMap.servo.get("leftBeacon");
        rightBeaconServo = hardwareMap.servo.get("rightBeacon");

        sensorRGB = (AMSColorSensorImpl) hardwareMap.colorSensor.get("color");

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        leftBeaconServo.scaleRange(LEFT_SERVO_MIN, LEFT_SERVO_HOME);
        rightBeaconServo.scaleRange(RIGHT_SERVO_MIN, RIGHT_SERVO_HOME);

        leftBeaconServo.setPosition(1);
        rightBeaconServo.setPosition(1);
        
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        gyro.calibrate();

        this.runUsingEncoders();
        this.resetEncoders();
    }

    @Override
    public void loop() {
        this.autoDrive(28, 1.0); //TODO - Use this.autoDrivePID(28);
        this.shootBall();
        this.runIntake(5000);
        this.shootBall();
        this.autoDrive(24, 1.0); //TODO - Use this.autoDrivePID(24);
        this.turn(-90, 1.0);
        /*
        this.autoDrivePID(43, 1.0);
        this.readBeacon(teamColor);
        this.autoDrivePID(6, 1.0);
        this.resetServos();
        this.autoDrivePID(-10, 1.0);
        this.turn(90, 1.0);
        this.autoDrivePID(36, 1.0);
        */
        requestOpModeStop();
    }
    
    @Override
    public void stop() {
    	this.setPower(0, 0);
    }
    
    private void runUsingEncoders() {
    	leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    	leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    	rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    	rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    	
    	shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private void resetEncoders() {
    	leftOffset = leftFrontMotor.getCurrentPosition();
    	rightOffset = rightFrontMotor.getCurrentPosition();
    	
    	shooterOffset = shooterMotor.getCurrentPosition();
    }
    
    /**
     * Set the power of the left and right motors
     * @param leftPower
     * @param rightPower
     */
    private void setPower(double leftPower, double rightPower) {
    	leftFrontMotor.setPower(leftPower - LEFT_POWER_OFFSET);
    	rightFrontMotor.setPower(rightPower - RIGHT_POWER_OFFSET);
        leftBackMotor.setPower(leftPower - LEFT_POWER_OFFSET);
    	rightBackMotor.setPower(rightPower - RIGHT_POWER_OFFSET);
    }

    /**
     * Method to drive distance with positive power
     * @param distance
     * @param power (positive)
     */
    private void autoDrive(double distance, double power) {
        if(power < 0) throw new IllegalArgumentException("power = " + power);
        if(distance < 0) {
            power = -power;
            distance = -distance;
        }
        this.resetEncoders();
        
        double distanceInTicks = (distance / INCHES_PER_TICK) + TICK_OFFSET;
        
        while(getLeftPosition() < distanceInTicks && getRightPosition() < distanceInTicks) {
            this.setPower(power, power);
        }
        this.setPower(0, 0);
        
        this.resetEncoders();
    }

    /**
     * Method to drive distance with PID control
     * @param distance
     * @param power (positive)
     */
    // FIXME - Get this working
    private final double PID_DRIVE_GAIN             = 0.5;
    private final double PID_DRIVE_TOLERANCE_INCHES = 0.5;
    private void autoDrivePID(double distance) {
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
     * @param power (positive)
     */
    // TODO - Test
    private void turn(int degrees, double power) {
        if(power < 0) throw new IllegalArgumentException("power = " + power);
        
        double distance = (degrees * (Math.PI / 180) * ROBOT_RADIUS) / (WHEEL_DIAMETER * Math.PI) * ENCODER_TICKS_PER_REV;
        
        if(degrees < 0) {
            power = -power;
            degrees = -degrees;
            distance = -distance;
        }

        while(getLeftPosition() < distance && getRightPosition() < -distance) {
            this.setPower(power, -power);
        }
    	this.setPower(0, 0);
    	
        this.resetEncoders();
    }

    /**
     * Shoots a ball
     */
    private void shootBall() {
        while(getShooterPosition() < SHOOTER_ROTATION) {
                shooterMotor.setPower(1);
            }
        shooterMotor.setPower(0);
        
        this.resetEncoders();
    }

    /**
     * Runs the intake for a specified amount of time
     * @param milliseconds
     */
    private void runIntake(int milliseconds) {
    	time.reset();
        while(time.milliseconds() < milliseconds) {
            intakeMotor.setPower(1);
        }
        intakeMotor.setPower(0);
    }
    
    // TODO - Implement line follower
    private void followLine() {
    	
    }

    /**
     * Reads the color on the beacon
     * @param teamColor
     */
    private void readBeacon(FtcColor teamColor) {
    	if(teamColor == FtcColor.RED && sensorRGB.red() >= RED_THRESHOLD && sensorRGB.blue() <= BLUE_THRESHOLD) {
    		leftBeaconServo.setPosition(LEFT_SERVO_MIN);
    	} else {
    		rightBeaconServo.setPosition(RIGHT_SERVO_MIN);
    	}
    }

    private int getLeftPosition() {
        return leftFrontMotor.getCurrentPosition() - leftOffset;
    }

    private int getRightPosition() {
        return rightFrontMotor.getCurrentPosition() - rightOffset;
    }

    private int getShooterPosition() {
        return shooterMotor.getCurrentPosition() - shooterOffset;
    }

    private void resetServos() {
        leftBeaconServo.setPosition(1);
        rightBeaconServo.setPosition(1);
    }
}

