package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime.Resolution;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Darren Kam on 9/28/2016.
 */
public class Team3Robot {

    public static enum FtcColor {
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
    
    final static int REGULATOR_TIME = 800;
    
    final static int DISTANCE_OFFSET      = 8;
    final static int SHOOTER_ROTATION 	  = 720;  // TODO - Find shooter ticks

    // TODO - Find offsets //
    //final static double LEFT_POWER_OFFSET  = 0.27;
    //final static double RIGHT_POWER_OFFSET = 0.40;
    final static int LED_PORT = 5;
    
    final static int LIFT_HOME = 0;
    final static int LIFT_UP = 500; // TODO - Find lift value
    
    final static double LIFT_POWER = 0.5;

    // Color sensor (TODO - Find thresholds) //
    final static int RED_THRESHOLD      = 495;
    final static int BLUE_THRESHOLD     = 1000;
    
    final static int GYRO_OFFSET = 17;
    
    // Light sensor threshold //
    final static double LIGHT_THRESHOLD = 0.2;
    
    final static double LINE_POWER = 0.3;   

    final static double BEACON_SPEED = -0.5;
    
    final static int WAIT_PERIOD = 250;
    
    // Field constants //
    final static double DISTANCE_TO_VORTEX = 26 + DISTANCE_OFFSET;
    final static double DISTANCE_TO_BEACON = 4 + DISTANCE_OFFSET;
    final static int RED_BEACON_DEGREES = 29;
    final static int BLUE_BEACON_DEGREES = 331;

    // TODO - Implement later
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.8;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.1;     // Larger is more responsive, but also less stable
    
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

    // Hardware declarations //
    DcMotor _rightFrontMotor;
    DcMotor _leftFrontMotor;
    DcMotor _rightBackMotor;
    DcMotor _leftBackMotor;

    DcMotor _intakeMotor;
    DcMotor _shooterMotor;
    
    DcMotor _leftLiftMotor;
    DcMotor _rightLiftMotor;

    Servo _rightBeaconServo;
    Servo _leftBeaconServo;

    Servo _regulatorServo;

    ColorSensor _sensorRGB;
    DeviceInterfaceModule _cdim;

    ModernRoboticsI2cGyro _gyro;

    LightSensor _lightSensor;
    
    ElapsedTime _time = new ElapsedTime(Resolution.MILLISECONDS);
    
    // OpMode members //
    HardwareMap hardwareMap = null;
    Telemetry telemetry = null;

    /************** OpMode methods **************/

    protected void init(HardwareMap hardwareMap, Telemetry telemetry) {
    	_teamColor = FtcColor.NONE;
    	
    	this.hardwareMap = hardwareMap;
    	this.telemetry = telemetry;

        _leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        _leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        _rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        _rightBackMotor = hardwareMap.dcMotor.get("rightRear");

        _leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        _leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        _intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        _shooterMotor = hardwareMap.dcMotor.get("shooter");
        
        _shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        
        _leftLiftMotor = hardwareMap.dcMotor.get("leftLift");
        _rightLiftMotor = hardwareMap.dcMotor.get("rightLift");
        
        _rightLiftMotor.setDirection(DcMotor.Direction.REVERSE);

        _leftBeaconServo = hardwareMap.servo.get("leftBeacon");
        _rightBeaconServo = hardwareMap.servo.get("rightBeacon");

        _leftBeaconServo.scaleRange(LEFT_SERVO_MIN, LEFT_SERVO_HOME);
        _rightBeaconServo.scaleRange(RIGHT_SERVO_MIN, RIGHT_SERVO_HOME);
        
        _leftBeaconServo.setPosition(1);
        _rightBeaconServo.setPosition(1);
        
        _regulatorServo = hardwareMap.servo.get("regulator");

        _regulatorServo.scaleRange(REGULATOR_SERVO_MIN, REGULATOR_SERVO_HOME);

        _regulatorServo.setPosition(1);

        _ledState = false;
        
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
        
        this.resetMotors();
        this.resetIntake();
        this.resetShooter();
        
        _time.reset();
    }

    protected void stop() {
    	this.setPower(0);
    }

    /************** Utility methods **************/

    private void runUsingEncoders() {
    	_leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        _intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        _leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    	leftPower = this.scaleInput(leftPower);
    	rightPower = this.scaleInput(rightPower);
    	
        _leftFrontMotor.setPower(leftPower);
        _leftBackMotor.setPower(leftPower);
        _rightFrontMotor.setPower(rightPower);
        _rightBackMotor.setPower(rightPower);
    }
    
    protected void liftRegulator() {
        _regulatorServo.setPosition(0);
        this.wait(REGULATOR_TIME);
        _regulatorServo.setPosition(1);
    }
    
    /**
     * Set the lift position to HOME or UP
     * @param position
     */
    protected void setLiftPosition(int position) {
    	if(position > 0 && _leftLiftMotor.getCurrentPosition() == 0) {
    		while(this.getLiftPosition() < position) {
    			_leftLiftMotor.setPower(LIFT_POWER);
    			_rightLiftMotor.setPower(LIFT_POWER);
    		}
    	} else {
    		while(this.getLiftPosition() > position) {
    			_leftLiftMotor.setPower(-LIFT_POWER);
    			_rightLiftMotor.setPower(-LIFT_POWER);
    		}
    	}
    }
    
    protected void setTeamColor(FtcColor teamColor) {
        _teamColor = teamColor;
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
    
    protected int getLiftPosition() {
    	return _leftLiftMotor.getCurrentPosition() - _liftOffset;
    }

    protected int getGyro() {
        return _gyro.getIntegratedZValue() - _gyroOffset;
    }

    protected double getLight() {
        return _lightSensor.getLightDetected() - _lightOffset;
    }
    
    // TODO - Fix for negative light
    protected boolean lineDetected() {
        return Math.abs(this.getLight()) > LIGHT_THRESHOLD;
    }
    
    private double getTime() {
    	return _time.time();
    }

    private FtcColor getBeaconColor() {
        FtcColor beaconColor;
        if(_sensorRGB.red() < RED_THRESHOLD && _sensorRGB.blue() > _sensorRGB.red()) {
            beaconColor = FtcColor.BLUE;
        } else {
            beaconColor = FtcColor.RED;
        }
        return beaconColor;
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

    // TODO - Figure out which to use
    protected void resetGyro() {
        _gyroOffset = _gyro.getIntegratedZValue();
        _gyro.resetZAxisIntegrator();
    }
    
    private void calibrateGyro() {
    	_gyro.calibrate();
    	while(_gyro.isCalibrating()) {
    		telemetry.addData("gyro", "calibrating");
    		telemetry.update();
    	}
    	telemetry.addData("gyro", _gyro.getHeading());
    	telemetry.update();
    }

    protected void wait(int milliseconds) {
        int initTime = (int) _time.milliseconds();
        while(_time.milliseconds() - initTime < milliseconds) {
        	this.setPower(0);
        }
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

        this.wait(Team3Robot.WAIT_PERIOD);
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

        this.wait(Team3Robot.WAIT_PERIOD);
    }

    private void driveToLine() {
    	_lightSensor.enableLed(true);
    	
        this.setPower(-LINE_POWER);
        
        while (!this.lineDetected()) {
            telemetry.addData("Light Sensor", this.getLight());
            telemetry.addData("Line detected", this.lineDetected());
            telemetry.update();
        }
        this.setPower(0);
        
        _lightSensor.enableLed(false);
    }

    private void alignToLine() {
        if(_teamColor == FtcColor.RED) {
            this.autoTurnInPlace(RED_BEACON_DEGREES, LINE_POWER);
        } else {
            this.autoTurnInPlace(BLUE_BEACON_DEGREES, LINE_POWER);
        }
    }
    
    /**
     * Scale input to scaling array
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

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - _gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        this.setPower(leftSpeed, rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
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

        this.resetMotors();

        double distanceInTicks = distance / INCHES_PER_TICK;
        
        if(distance < 0) {
            while(this.getLeftPosition() > -distanceInTicks && this.getRightPosition() > -distanceInTicks) {
                this.setPower(-leftPower, -rightPower);
            }
        } else {
            while(this.getLeftPosition() < distanceInTicks && this.getRightPosition() < distanceInTicks) {
                this.setPower(leftPower, rightPower);
            }
        }
        this.setPower(0);

        this.wait(Team3Robot.WAIT_PERIOD);
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    /*public void autoGyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance / INCHES_PER_TICK);
            newLeftTarget = this.getLeftPosition() + moveCounts;
            newRightTarget = this.getRightPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            _leftFrontMotor.setTargetPosition(newLeftTarget);
            _rightFrontMotor.setTargetPosition(newRightTarget);
            _leftBackMotor.setTargetPosition(newLeftTarget);
            _rightBackMotor.setTargetPosition(newRightTarget);

            _leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            _rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            _leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            _rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            this.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (_leftFrontMotor.isBusy() && _rightFrontMotor.isBusy() && _leftBackMotor.isBusy() && _rightBackMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                this.setPower(leftSpeed, rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      _leftFrontMotor.getCurrentPosition(),
                        _rightFrontMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            this.setPower(0);

            // Turn off RUN_TO_POSITION
            _leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }*/

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void autoGyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (!onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to turn degrees with specified power
     * @param degrees
     * @param power
     */
    protected void autoTurnInPlace(int degrees, double power) {
        if(degrees < -360 || degrees > 360) throw new IllegalArgumentException("degrees = " + degrees);
        if(power < 0) throw new IllegalArgumentException("power = " + power);

        this.resetGyro();

        while(this.getGyro() < degrees - GYRO_OFFSET) {
            this.setPower(power, -power);
            telemetry.addData("gyro", this.getGyro());
            telemetry.update();
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
        this.autoDriveDistance(DISTANCE_TO_BEACON, -BEACON_SPEED);
    }
}

