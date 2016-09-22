/* Copyright (c) 2014 Qualcomm Technologies Inc
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public abstract class SuperK9Base extends OpMode {

    // dead reckoning information //
    final static int    ENCODER_TICKS_PER_REV = 1120; // Neverest 40
    final static int    WHEEL_DIAMETER        = 6;    // inches / REV
    final static double INCHES_PER_TICK       = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;

    // servo position information //
    final static double BUTTON_SERVO_POS_LEFT   = 0.34;
    final static double BUTTON_SERVO_POS_CENTER = 0.54;
    final static double BUTTON_SERVO_POS_RIGHT  = 0.74;

    protected enum ButtonServoPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    final static double MAN_SERVO_POS_PARK   = 0.0;
    final static double MAN_SERVO_POS_HOME   = 0.18;
    final static double MAN_SERVO_POS_HOVER  = 0.48;
    final static double MAN_SERVO_POS_DEPLOY = 0.78;

    protected enum ManServoPosition {
        PARK,
        HOME,
        HOVER,
        DEPLOY
    }

    final static double PLOW_POWER_MIN = -0.25;
    final static double PLOW_POWER_MAX = 0.5;
    final static double PLOW_POWER_MIN2 = -0.15;
    final static double PLOW_POWER_MAX2 = 0.25;

    final static double DOZER_POWER_MIN = -0.35;
    final static double DOZER_POWER_MAX = 0.5;

    final static double LAUNCH_MOTOR_POWER = 0.5;
    final static double LAUNCH_LOOP_COUNT = 5;

    // color sensor information //
    final static double HUE_THRESHOLD_RED  = 25.0;
    final static double HUE_THRESHOLD_BLUE = 200.0;

    protected enum FtcColor {
        RED,
        BLUE,
        NONE
    }

    // trigger configuration //
    final static double TRIGGER_LEFT_POS_IN  = 0.62;
    final static double TRIGGER_LEFT_POS_OUT = 0.0;

    final static double TRIGGER_RIGHT_POS_IN  = 0.4;
    final static double TRIGGER_RIGHT_POS_OUT = 1.0;

    final static double TRIGGER_8898_OFFSET = 0.15;

    // light sensor configuration //
    final static double INNER_LIGHT_THRESHOLD = 0.3; // note: these should be positive //
    final static double OUTER_LIGHT_THRESHOLD = 0.3;

    // hardware instances //
    DcMotor _motorRightFront;
    DcMotor _motorRightRear;
    DcMotor _motorLeftFront;
    DcMotor _motorLeftRear;
    DcMotor _winchMotor;
    DcMotor _launchMotor;
    DcMotor _plowMotor2;

    Servo _launchServo;
    Servo _manServo;
    Servo _plowMotor;
    Servo _dozerMotor;

    Servo _leftTrigger;
    Servo _rightTrigger;

    DeviceInterfaceModule _cdim;
    ColorSensor _sensorRGB;
    DigitalChannel _ledColorSensor;
    LightSensor _lightInner;
    DigitalChannel _ledInnerRed, _ledInnerBlue;
    LightSensor _lightOuter;
    DigitalChannel _ledOuterRed, _ledOuterBlue;
    ModernRoboticsI2cGyro _gyro;

    final ElapsedTime _time = new ElapsedTime();

    int _leftEncoderOffset  = 0;
    int _rightEncoderOffset = 0;
    double _lightInnerOffset = 0.0;
    double _lightOuterOffset = 0.0;
    int _gyroHeadingOffset = 0;

    protected enum TeamNumber {
        TEAM_8740,
        TEAM_8741,
        TEAM_8898
    }
    private TeamNumber _teamNumber;

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {
        // use "jumper" hardware to determine team number //
        try {
            hardwareMap.digitalChannel.get("robot_8741");
            _teamNumber = TeamNumber.TEAM_8741;
        } catch(Exception e) {
            try {
                hardwareMap.digitalChannel.get("robot_8898");
                _teamNumber = TeamNumber.TEAM_8898;
            } catch(Exception e2) {
                // no jumper set //
                _teamNumber = TeamNumber.TEAM_8740;
            }
        }

        _motorRightFront = hardwareMap.dcMotor.get("rightFront");
        _motorRightRear  = hardwareMap.dcMotor.get("rightRear");
        _motorLeftFront = hardwareMap.dcMotor.get("leftFront");
        _motorLeftRear = hardwareMap.dcMotor.get("leftRear");

        _motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        _motorRightRear.setDirection(DcMotor.Direction.REVERSE);

        _winchMotor = hardwareMap.dcMotor.get("winchMotor");
        if(_teamNumber == TeamNumber.TEAM_8898) {
            // reverse winch on 8898 //
            _winchMotor.setDirection(DcMotor.Direction.REVERSE);
        }
        _winchMotor.setPowerFloat();

        _launchServo = hardwareMap.servo.get("launchServo");
        _launchServo.setDirection(Servo.Direction.REVERSE);
        _launchMotor = hardwareMap.dcMotor.get("launchMotor");
        this.setLaunchReleasePower(0);

        _cdim = hardwareMap.deviceInterfaceModule.get("dim");
        _sensorRGB = hardwareMap.colorSensor.get("color");
        _ledColorSensor = hardwareMap.digitalChannel.get("ledColor");
        _ledColorSensor.setMode(DigitalChannelController.Mode.OUTPUT);
        this.setColorSensorLED(false);

        _lightOuter = hardwareMap.lightSensor.get("lightOuter");
        _ledOuterRed = hardwareMap.digitalChannel.get("ledOuterRed");
        _ledOuterRed.setMode(DigitalChannelController.Mode.OUTPUT);
        _ledOuterBlue = hardwareMap.digitalChannel.get("ledOuterBlue");
        _ledOuterBlue.setMode(DigitalChannelController.Mode.OUTPUT);
        this.setOuterLightLEDColor(FtcColor.NONE);
        _lightInner  = hardwareMap.lightSensor.get("lightInner");
        _ledInnerRed = hardwareMap.digitalChannel.get("ledInnerRed");
        _ledInnerRed.setMode(DigitalChannelController.Mode.OUTPUT);
        _ledInnerBlue = hardwareMap.digitalChannel.get("ledInnerBlue");
        _ledInnerBlue.setMode(DigitalChannelController.Mode.OUTPUT);
        this.setInnerLightLEDColor(FtcColor.NONE);

        _manServo = hardwareMap.servo.get("manServo");
        this.setManServoPosition(ManServoPosition.HOME);

        _rightTrigger = hardwareMap.servo.get("rightTrigger");
        _rightTrigger.setDirection(Servo.Direction.REVERSE);
        this.setRightTriggerDeployed(false);
        _leftTrigger  = hardwareMap.servo.get("leftTrigger");
        _leftTrigger.setDirection(Servo.Direction.REVERSE);
        this.setLeftTriggerDeployed(false);

        _plowMotor = hardwareMap.servo.get("plowMotor");
        try {
            _plowMotor2 = hardwareMap.dcMotor.get("plowMotor2");
        } catch(Exception e) {
        }
        this.setPlowPower(0);

        _dozerMotor = hardwareMap.servo.get("dozerMotor");
        this.setDozerPower(0);

        _gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        _gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        this.k9Init();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Gyro Heading", _gyro.isCalibrating()? "calibrating": _gyro.getHeading());
        //telemetry.addData("Gyro Integrator", _gyro.isCalibrating()? "calibrating": _gyro.getIntegratedZValue());
        this.k9InitLoop();
    }

    /*
     * This method will be called on start
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void start() {
        _time.reset();
        this.k9Start();
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        //telemetry.addData("Text", "*** Robot Data ***");
        //this.launchMotorLoop();
        this.k9Loop();

        /*
         * Send telemetry data back to driver station. Note that if we are using
         * a legacy NXT-compatible motor controller, then the getPower() method
         * will return a null value. The legacy NXT-compatible motor controllers
         * are currently write only.
         */
        telemetry.addData("Left encoder", String.format("%.2f", this.getLeftPositionInches()));
        telemetry.addData("Right encoder", String.format("%.2f", this.getRightPositionInches()));
        //telemetry.addData("Plow power", String.format("%.2f", this.getPlowPower()));
        //telemetry.addData("Dozer power", String.format("%.2f", this.getDozerPower()));
        telemetry.addData("Gyro Heading", _gyro.isCalibrating()? "calibrating": this.getGyroHeading());
        //telemetry.addData("Gyro Integrator", _gyro.isCalibrating()? "calibrating": _gyro.getIntegratedZValue());
        telemetry.addData("Team #", this.getTeamNumber());
        //telemetry.addData("Color (Hue)", String.format("%s (%.2f)", this.getColorSensor(), this.getColorSensorHue()));
        //telemetry.addData("ODS", String.format("%.2f", this.getODSLight()));
        //telemetry.addData("Lego", String.format("%.2f", this.getLegoLight()));
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {
        this.k9Stop();
    }

    protected void k9Init() { }

    protected void k9InitLoop() { }

    protected void k9Start() { }

    protected void k9Loop() { }

    protected void k9Stop() { }

    protected TeamNumber getTeamNumber() {
        return _teamNumber;
    }

    void setPowerArcade(double moveValue, double rotateValue, boolean squaredInputs) {
        // local variables to hold the computed PWM values for the motors
        double leftMotorOutput;
        double rightMotorOutput;

        moveValue = Range.clip(moveValue, -1.0, 1.0);
        rotateValue = Range.clip(rotateValue, -1.0, 1.0);

        if (squaredInputs) {
            // square the inputs (while preserving the sign) to increase fine control while permitting full power
            if (moveValue >= 0.0) {
                moveValue = (moveValue * moveValue);
            } else {
                moveValue = -(moveValue * moveValue);
            }
            if (rotateValue >= 0.0) {
                rotateValue = (rotateValue * rotateValue);
            } else {
                rotateValue = -(rotateValue * rotateValue);
            }
        }

        if (moveValue > 0.0) {
            if (rotateValue > 0.0) {
                leftMotorOutput = moveValue - rotateValue;
                rightMotorOutput = Math.max(moveValue, rotateValue);
            } else {
                leftMotorOutput = Math.max(moveValue, -rotateValue);
                rightMotorOutput = moveValue + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                leftMotorOutput = -Math.max(-moveValue, rotateValue);
                rightMotorOutput = moveValue + rotateValue;
            } else {
                leftMotorOutput = moveValue - rotateValue;
                rightMotorOutput = -Math.max(-moveValue, -rotateValue);
            }
        }
        this.setPower(leftMotorOutput, rightMotorOutput);
    }

    protected void setPowerScaled(double leftPower, double rightPower) {

        rightPower = Range.clip(rightPower, -1, 1);
        leftPower  = Range.clip(leftPower, -1, 1);

        rightPower = scaleInput(rightPower);
        leftPower =  scaleInput(leftPower);

        this.setPower(leftPower, rightPower);
    }

    protected void setPower(double leftPower, double rightPower) {
        // write the values to the motors
        _motorRightFront.setPower(rightPower);
        _motorRightRear.setPower(rightPower);
        _motorLeftFront.setPower(leftPower);
        _motorLeftRear.setPower(leftPower);
    }

    protected int getLeftEncoder() {
        return _motorLeftFront.getCurrentPosition() - _leftEncoderOffset;
    }

    protected double getLeftPositionInches() {
        return this.getLeftEncoder() * INCHES_PER_TICK;
    }

    protected int getRightEncoder() {
        return _motorRightFront.getCurrentPosition() - _rightEncoderOffset;
    }

    protected double getRightPositionInches() {
        return this.getRightEncoder() * INCHES_PER_TICK;
    }

    protected void resetEncoders() {
        //this.setEncoderMode(DcMotorController.RunMode.RESET_ENCODERS);
        _leftEncoderOffset  = _motorLeftFront.getCurrentPosition();
        _rightEncoderOffset = _motorRightFront.getCurrentPosition();
    }

    protected boolean areEncodersReset() {
        return this.getLeftEncoder() == 0 && this.getRightEncoder() == 0;
    }

    protected void runWithEncoders() {
        this.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void runWithoutEncoders() {
        this.setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void runToPosition(double leftInches, double rightInches, double power) {
        int leftTicks  = (int) ((leftInches  + _leftEncoderOffset)  / INCHES_PER_TICK);
        int rightTicks = (int) ((rightInches + _rightEncoderOffset) / INCHES_PER_TICK);

        this.setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        _motorLeftFront.setTargetPosition(leftTicks);
        _motorRightFront.setTargetPosition(rightTicks);
        _motorLeftRear.setTargetPosition(leftTicks);
        _motorRightRear.setTargetPosition(rightTicks);
        this.setPower(this.sign(leftInches) * power, this.sign(rightInches) * power);
    }

    protected boolean isRunning() {
        return _motorLeftFront.isBusy() || _motorRightFront.isBusy();
    }


    protected ManServoPosition getManServoPosition() {
        double pos = _manServo.getPosition();
        return  pos == MAN_SERVO_POS_DEPLOY? ManServoPosition.DEPLOY:
                pos == MAN_SERVO_POS_HOME? ManServoPosition.HOME:
                        ManServoPosition.PARK;
    }

    protected void setManServoPosition(ManServoPosition pos) {
        switch (pos) {
            case HOME:
                _manServo.setPosition(MAN_SERVO_POS_HOME);
                break;
            case DEPLOY:
                _manServo.setPosition(MAN_SERVO_POS_DEPLOY);
                break;
            case HOVER:
                _manServo.setPosition(MAN_SERVO_POS_HOVER);
                break;
            case PARK:
                _manServo.setPosition(MAN_SERVO_POS_PARK);
                break;
        }
    }

    protected boolean getLeftTriggerDeployed() {
        return _leftTrigger.getPosition() == TRIGGER_LEFT_POS_OUT;
    }

    protected void setLeftTriggerDeployed(boolean out) {
        double offset = _teamNumber == TeamNumber.TEAM_8898? -TRIGGER_8898_OFFSET: 0;
        _leftTrigger.setPosition(out ? TRIGGER_LEFT_POS_OUT: TRIGGER_LEFT_POS_IN + offset);
    }

    protected boolean getRightTriggerDeployed() {
        return _rightTrigger.getPosition() == TRIGGER_RIGHT_POS_OUT;
    }

    protected void setRightTriggerDeployed(boolean out) {
        double offset = _teamNumber == TeamNumber.TEAM_8898? TRIGGER_8898_OFFSET: 0;
        _rightTrigger.setPosition(out ? TRIGGER_RIGHT_POS_OUT: TRIGGER_RIGHT_POS_IN + offset);
    }

    protected double getPlowPower() {
        return (_plowMotor.getPosition() * 2) - 1;
    }

    // negative is deploy, positive retract //
    protected void setPlowPower(double power) {
        double xpower = (Range.clip(power, PLOW_POWER_MIN, PLOW_POWER_MAX) + 1) / 2;
        _plowMotor.setPosition(xpower);
        if(_plowMotor2 != null) {
            _plowMotor2.setPower(Range.clip(power, PLOW_POWER_MIN2, PLOW_POWER_MAX2));
        }
    }

    protected double getDozerPower() {
        return (_dozerMotor.getPosition() * 2) - 1;
    }

    // positive is deploy, negative retract //
    protected void setDozerPower(double power) {
        power = (Range.clip(power, DOZER_POWER_MIN, DOZER_POWER_MAX) + 1) / 2;
        _dozerMotor.setPosition(power);
    }

    protected double getWinchPower() {
        return _winchMotor.getPower();
    }

    protected void setWinchPower(double power) {
        power = Range.clip(power, 0, 1.0);
        _winchMotor.setPower(power);
    }

    protected void setLaunchReleasePower(double power) {
        power = Range.clip(power, -1.0, 1.0);
        _launchServo.setPosition(0.5 + power / 2);
        if(power != 0){
            _launchMotor.setPower(power);
        } else {
            _launchMotor.setPowerFloat();
        }
    }

    protected void calibrateGyro() {
        _gyro.calibrate();
    }

    protected boolean isGyroCalibrating() {
        return _gyro.isCalibrating();
    }

    protected int getGyroHeading() {
        return _gyro.getIntegratedZValue() - _gyroHeadingOffset;
    }

    protected void resetGyroHeading() {
        _gyroHeadingOffset = _gyro.getIntegratedZValue();
    }

    protected float getColorSensorHue() {
        float hsvValues[] = {0F,0F,0F};

        Color.RGBToHSV((_sensorRGB.red() * 255) / 800, (_sensorRGB.green() * 255) / 800, (_sensorRGB.blue() * 255) / 800, hsvValues);
        return hsvValues[0];
    }

    protected FtcColor getColorSensor() {
        float hue = this.getColorSensorHue();
        return hue <= HUE_THRESHOLD_RED? FtcColor.RED: hue >= HUE_THRESHOLD_BLUE? FtcColor.BLUE: FtcColor.NONE;
    }

    protected boolean getColorSensorLED() {
        return _ledColorSensor.getState();
    }

    protected void setColorSensorLED(boolean enabled) {
        _ledColorSensor.setState(enabled);
    }

    protected boolean isInnerLineDetected() {
        return Math.abs(this.getLightInner()) > INNER_LIGHT_THRESHOLD;
    }

    protected double getLightInner() {
        return _lightInner.getLightDetected() - _lightInnerOffset;
    }

    protected void setInnerLightLEDColor(FtcColor color) {
        // active low, turn on opposite color //
        _ledInnerRed.setState(color == FtcColor.RED || color == FtcColor.NONE);
        _ledInnerBlue.setState(color == FtcColor.BLUE || color == FtcColor.NONE);
    }

    protected boolean isOuterLineDetected() {
        return Math.abs(this.getLightOuter()) > OUTER_LIGHT_THRESHOLD;
    }

    protected double getLightOuter() {
        return _lightOuter.getLightDetected() - _lightOuterOffset;
    }

    protected void setOuterLightLEDColor(FtcColor color) {
        // active low, turn on opposite color //
        _ledOuterRed.setState(color == FtcColor.RED || color == FtcColor.NONE);
        _ledOuterBlue.setState(color == FtcColor.BLUE || color == FtcColor.NONE);
    }

    protected void resetLightSensors() {
        _lightInnerOffset = _lightInner.getLightDetected();
        _lightOuterOffset = _lightOuter.getLightDetected();
    }

    protected int sign(double value) {
        return value > 0? 1: value < 0? -1: 0;
    }

    private void setEncoderMode(DcMotor.RunMode mode) {
        _motorLeftFront.setMode(mode);
        _motorRightFront.setMode(mode);
        _motorLeftRear.setMode(mode);
        _motorRightRear.setMode(mode);
    }

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
     *  Auto Command inner state machine. Use these for actions that require continually testing
     *  sensors or persistent state between loop invocations.
     */
    private enum AutoCommandState {
        NONE,
        DRIVE,
        PIVOT_TURN,
        IN_PLACE_TURN,
        PID_DRIVE,
        PID_TURN,
        GYRO_DRIVE,
        GYRO_TURN,
        WAIT,
        DRIVE_TO_LINE,
        TURN_TO_LINE
    }
    private AutoCommandState _commandState = AutoCommandState.NONE;

    protected double getTime() {
        return _time.time();
    }

    protected void autoEnd() {
        _commandState = AutoCommandState.NONE;
    }

    protected boolean autoDriveDistance(double inches, double speed) {
        if(speed < 0) throw new IllegalArgumentException("speed: " + speed);
        speed = Range.clip(speed, 0, 1.0);
        switch(_commandState) {
            case NONE:
                this.resetEncoders();
                this.setPower(0, 0);
                _commandState = AutoCommandState.DRIVE;
                break;
            case DRIVE:
                this.runWithEncoders();
                this.setPower(this.sign(inches) * speed, this.sign(inches) * speed);
                // check if we are at target distance //
                double left  = this.getLeftPositionInches();
                double right = this.getRightPositionInches();
                if(  (inches >= 0 && left >= inches && right >= inches)
                        || (inches < 0  && left <= inches && right <= inches))
                {
                    this.setPower(0, 0);
                    _commandState = AutoCommandState.NONE;
                    return true;
                }
                break;
            default:
                throw new IllegalStateException("drive called without ending previous command: " + _commandState);
        }
        return false;
    }

    // use PID control to move a precise distance //
    private final double PID_DRIVE_GAIN             = 0.5;
    private final double PID_DRIVE_TOLERANCE_INCHES = 0.5;
    protected boolean autoDriveDistancePID(double inches, double speed) {
        if(speed < 0) throw new IllegalArgumentException("speed: " + speed);
        speed = Range.clip(speed, 0, 1.0);
        switch(_commandState) {
            case NONE:
                this.resetEncoders();
                this.setPower(0, 0);
                _commandState = AutoCommandState.PID_DRIVE;
                break;
            case PID_DRIVE:
                this.runWithEncoders();
                // get distances //
                double left  = this.getLeftPositionInches();
                double right = this.getRightPositionInches();

                double error = inches - left; // use left pos for now //
                double power = Range.clip(PID_DRIVE_GAIN * error, -1.0, 1.0);
                this.setPower(power, power);
                if(Math.abs(error) < PID_DRIVE_TOLERANCE_INCHES) {
                    this.setPower(0, 0);
                    _commandState = AutoCommandState.NONE;
                    return true;
                }
                break;
            default:
                throw new IllegalStateException("pid drive called without ending previous command: " + _commandState);
        }
        return false;
    }

    // negative is clockwise, positive is counterclockwise //
    protected boolean autoTurnPivot(double inches, double speed) {
        if(speed < 0) throw new IllegalArgumentException("speed: " + speed);
        speed = Range.clip(speed, 0, 1.0);
        switch(_commandState) {
            case NONE:
                this.resetEncoders();
                this.setPower(0, 0);
                _commandState = AutoCommandState.PIVOT_TURN;
                break;
            case PIVOT_TURN:
                this.runWithEncoders();
                double current;
                if(inches >= 0) {
                    this.setPower(0, speed);
                    current = this.getRightPositionInches();
                } else {
                    this.setPower(speed, 0);
                    current = this.getLeftPositionInches();
                }
                if(current >= Math.abs(inches)) {
                    this.setPower(0, 0);
                    _commandState = AutoCommandState.NONE;
                    return true;
                }
                break;
            default:
                throw new IllegalStateException("pivot turn called without ending previous command: " + _commandState);
        }
        return false;
    }

    // negative is clockwise, positive is counterclockwise //
    protected boolean autoTurnInPlace(double inches, double speed) {
        if(speed < 0) throw new IllegalArgumentException("speed: " + speed);
        speed = Range.clip(speed, 0, 1.0);
        switch(_commandState) {
            case NONE:
                this.resetEncoders();
                this.setPower(0, 0);
                _commandState = AutoCommandState.IN_PLACE_TURN;
                break;
            case IN_PLACE_TURN:
                this.runWithEncoders();
                double current;
                if(inches >= 0) {
                    this.setPower(-speed, speed);
                    current = this.getRightPositionInches();
                } else {
                    this.setPower(speed, -speed);
                    current = this.getLeftPositionInches();
                }
                if(current >= Math.abs(inches)) {
                    this.setPower(0, 0);
                    _commandState = AutoCommandState.NONE;
                    return true;
                }
                break;
            default:
                throw new IllegalStateException("turn in place called without ending previous command: " + _commandState);
        }
        return false;
    }

    // negative is clockwise, positive is counterclockwise //
    private final double PID_TURN_GAIN             = 0.5;
    private final double PID_TURN_TOLERANCE_INCHES = 0.5;
    protected boolean autoTurnInPlacePID(double inches, double speed) {
        if(speed < 0) throw new IllegalArgumentException("speed: " + speed);
        speed = Range.clip(speed, 0, 1.0);
        switch(_commandState) {
            case NONE:
                this.resetEncoders();
                this.setPower(0, 0);
                _commandState = AutoCommandState.PID_TURN;
                break;
            case PID_TURN:
                this.runWithEncoders();
                // get distances //
                double left  = this.getLeftPositionInches();
                double right = this.getRightPositionInches();

                double error = inches - right;
                double power = Range.clip(PID_TURN_GAIN * error, -1.0, 1.0);
                this.setPower(-power, power);
                if(Math.abs(error) < PID_TURN_TOLERANCE_INCHES) {
                    this.setPower(0, 0);
                    _commandState = AutoCommandState.NONE;
                    return true;
                }
                break;
            default:
                throw new IllegalStateException("pid turn called without ending previous command: " + _commandState);
        }
        return false;
    }

    // use a gyro to drive straight for certain distance //
    // FIXME: does not work, should use PID for position also //
    private final double GYRO_DRIVE_GAIN = 0.025;
    protected boolean autoDriveDistanceGyro(double inches, double speed) {
        if(speed < 0) throw new IllegalArgumentException("speed: " + speed);
        speed = Range.clip(speed, 0, 1.0);
        if(_gyro == null) {
            throw new IllegalStateException("Gyro is not present");
        }
        switch(_commandState) {
            case NONE:
                this.resetEncoders();
                this.resetGyroHeading();
                this.setPower(0, 0);
                _commandState = AutoCommandState.GYRO_DRIVE;
                break;
            case GYRO_DRIVE:
                this.runWithEncoders();
                // get heading and normalize to +/- 180 //
                double heading = _gyro.getHeading();
                if(heading > 180) heading = heading - 360;
                double correction = (0 - heading) * GYRO_DRIVE_GAIN;
                this.setPower(this.sign(inches) * speed + correction, this.sign(inches) * speed - correction);
                // check if we are at target distance //
                double left  = this.getLeftPositionInches();
                double right = this.getRightPositionInches();
                if(  (inches >= 0 && left >= inches && right >= inches)
                        || (inches < 0  && left <= inches && right <= inches))
                {
                    this.setPower(0, 0);
                    _commandState = AutoCommandState.NONE;
                    return true;
                }
                break;
            default:
                throw new IllegalStateException("drive called without ending previous command: " + _commandState);
        }
        return false;
    }

    // negative is clockwise, positive is counterclockwise //
    private final double GYRO_TURN_P_GAIN       = 0.01;
    private final double GYRO_TURN_D_GAIN       = 0.1;
    private final double GYRO_TOLERANCE_DEGREES = 1;
    private double _lastGyroTurnError = 0;
    protected boolean autoTurnInPlaceGyro(double degrees, double speed) {
        if(Math.abs(degrees) > 180) throw new IllegalArgumentException("degrees: " + degrees);
        if(speed < 0) throw new IllegalArgumentException("speed: " + speed);
        speed = Range.clip(speed, 0, 1.0);
        if(_gyro == null) {
            throw new IllegalStateException("Gyro is not present");
        }
        switch(_commandState) {
            case NONE:
                this.setPower(0, 0);
                this.resetGyroHeading();
                _commandState = AutoCommandState.GYRO_TURN;
                break;
            case GYRO_TURN:
                this.runWithEncoders();
                // get heading and normalize //
                double error = degrees - this.getGyroHeading();
                double d_error = (error - _lastGyroTurnError);
                double power = Range.clip(error * GYRO_TURN_P_GAIN + d_error * GYRO_TURN_D_GAIN, -speed, speed);
                //DbgLog.msg("error: " + error + ", d_error: " + d_error + ", power: " + power);
                if(Math.abs(error) < GYRO_TOLERANCE_DEGREES) {
                    _commandState = AutoCommandState.NONE;
                    return true;
                } else {
                    this.setPower(-power, power);
                }
                _lastGyroTurnError = error;
                break;
            default:
                throw new IllegalStateException("gyro turn called without ending previous command: " + _commandState);
        }
        return false;
    }

    // wait for a specific number of seconds //
    private double _autoWaitStart = 0;
    protected boolean autoWaitSeconds(double seconds) {
        if(seconds < 0) throw new IllegalArgumentException("seconds: " + seconds);
        switch(_commandState) {
            case NONE:
                _autoWaitStart = this.getTime();
                _commandState  = AutoCommandState.WAIT;
                break;
            case WAIT:
                if(this.getTime() > _autoWaitStart + seconds) {
                    _commandState = AutoCommandState.NONE;
                    return true;
                }
                break;
            default:
                throw new IllegalStateException("wait called without ending previous command: " + _commandState);
        }
        return false;
    }

    // positive is forward, negative is reverse //
    protected boolean autoDriveToLine(double speed) {
        speed = Range.clip(speed, -1.0, 1.0);
        switch(_commandState) {
            case NONE:
                this.setPower(0, 0);
                _commandState = AutoCommandState.DRIVE_TO_LINE;
                break;
            case DRIVE_TO_LINE:
                this.runWithEncoders();
                this.setPower(speed, speed);
                if(this.isInnerLineDetected()) {
                    this.setPower(0, 0);
                    _commandState = AutoCommandState.NONE;
                    return true;
                }
                break;
            default:
                throw new IllegalStateException("drive to line called without ending previous command: " + _commandState);
        }
        return false;
    }

    // positive is counter-clockwise, negative is clockwise //
    protected boolean autoTurnToLine(double speed) {
        speed = Range.clip(speed, -1.0, 1.0);
        switch(_commandState) {
            case NONE:
                this.setPower(0, 0);
                _commandState = AutoCommandState.TURN_TO_LINE;
                break;
            case TURN_TO_LINE:
                this.runWithEncoders();
                this.setPower(-speed, speed);
                if(this.isOuterLineDetected()) {
                    this.setPower(0, 0);
                    _commandState = AutoCommandState.NONE;
                    return true;
                }
                break;
            default:
                throw new IllegalStateException("turn to line called without ending previous command: " + _commandState);
        }
        return false;
    }

}