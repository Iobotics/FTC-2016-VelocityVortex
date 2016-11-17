package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.SuperK9Base.FtcColor.NONE;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name="TestRobot: Autonomous", group="TestRobot")
@Disabled
public class TestRobotAuto extends OpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
  //  ModernRoboticsI2cGyro _gyro;

    final static int ENCODER_TICKS_PER_REV = 1120;
    final static int WHEEL_DIAMETER = 6;
    final static double INCHES_PER_TICK = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;

    ColorSensor cs1;
    int cs2;

    int _leftEncoderOffset;
    int _rightEncoderOffset;
    int _gyroHeadingOffset;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");
//        _gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
//      _gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        //cs1= hardwareMap.colorSensor.get("Lego");

//        double turn = _gyro.getRotationFraction();
//        telemetry.addData("Status", _gyro.status());
//        telemetry.update();
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
}

    @Override
    public void loop() {

        frontLeftMotor.setTargetPosition(targetPosition(12,6));
        frontRightMotor.setTargetPosition(targetPosition(12,6));
        backLeftMotor.setTargetPosition(targetPosition(12,6));
        backRightMotor.setTargetPosition(targetPosition(12,6));

        while(0 <= frontLeftMotor.getCurrentPosition())
        {
            frontLeftMotor.setPower(0.3);
            frontRightMotor.setPower(0.3);
            backLeftMotor.setPower(0.3);
            backRightMotor.setPower(0.3);
        }
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);


        frontLeftMotor.setTargetPosition(targetPosition(24 * Math.PI/4,6));
        frontRightMotor.setTargetPosition(targetPosition(24 * Math.PI/4,6));
        backLeftMotor.setTargetPosition(targetPosition(24 * Math.PI/4,6));
        backRightMotor.setTargetPosition(targetPosition(24 * Math.PI/4,6));

        while(0 <= frontLeftMotor.getCurrentPosition())
        {
            frontLeftMotor.setPower(0.3);
            frontRightMotor.setPower(-0.3);
            backLeftMotor.setPower(0.3);
            backRightMotor.setPower(-0.3);
        }
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);

        frontLeftMotor.setTargetPosition(targetPosition(12,6));
        frontRightMotor.setTargetPosition(targetPosition(12,6));
        backLeftMotor.setTargetPosition(targetPosition(12,6));
        backRightMotor.setTargetPosition(targetPosition(12,6));

        while(0 <= frontLeftMotor.getCurrentPosition())
        {
            frontLeftMotor.setPower(0.3);
            frontRightMotor.setPower(0.3);
            backLeftMotor.setPower(0.3);
            backRightMotor.setPower(0.3);
        }
        stop();
    }

    @Override
    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private int targetPosition(double distanceToGo, double wheelDiameter){
        double push;
        double circumfrence;
        circumfrence= wheelDiameter * Math.PI;
        push= distanceToGo / circumfrence;
        push= push * 1120;
        return (int) push;
    }
/*

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

    protected int getLeftEncoder() {
        return frontLeftMotor.getCurrentPosition() - _leftEncoderOffset;
    }

    protected double getLeftPositionInches() {
        return this.getLeftEncoder() * INCHES_PER_TICK;
    }

    protected int getRightEncoder() {
        return frontRightMotor.getCurrentPosition() - _rightEncoderOffset;
    }

    protected double getRightPositionInches() {
        return this.getRightEncoder() * INCHES_PER_TICK;
    }

    protected void resetEncoders() {
        //this.setEncoderMode(DcMotorController.RunMode.RESET_ENCODERS);
        _leftEncoderOffset = frontLeftMotor.getCurrentPosition();
        _rightEncoderOffset = frontRightMotor.getCurrentPosition();
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
        int leftTicks = (int) ((leftInches + _leftEncoderOffset) / INCHES_PER_TICK);
        int rightTicks = (int) ((rightInches + _rightEncoderOffset) / INCHES_PER_TICK);

        this.setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setTargetPosition(leftTicks);
        frontRightMotor.setTargetPosition(rightTicks);
        backLeftMotor.setTargetPosition(leftTicks);
        backRightMotor.setTargetPosition(rightTicks);
        this.setPower(this.sign(leftInches) * power, this.sign(rightInches) * power);
    }

    private void setEncoderMode(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        backLeftMotor.setMode(mode);
        backRightMotor.setMode(mode);
    }

    protected void setPower(double leftPower, double rightPower) {
        // write the values to the motors
        frontRightMotor.setPower(rightPower);
        backRightMotor.setPower(rightPower);
        frontLeftMotor.setPower(leftPower);
        backLeftMotor.setPower(leftPower);
    }

    protected int sign(double value) {
        return value > 0 ? 1 : value < 0 ? -1 : 0;
    }

    private enum AutoCommandState {
        NONE,
        GYRO_TURN,
        GYRO_DRIVE,
    }

    private SuperK9Base.AutoCommandState _commandState = SuperK9Base.AutoCommandState.NONE;

    protected void autoDriveDistanceGyro(double inches, double speed) {
        if (speed < 0) throw new IllegalArgumentException("speed: " + speed);
        speed = Range.clip(speed, 0, 1.0);
        if (_gyro == null) {
            throw new IllegalStateException("Gyro is not present");
        }
        switch(_commandState) {
            case NONE:
                this.resetEncoders();
                this.resetGyroHeading();
                this.setPower(0, 0);
                _commandState = SuperK9Base.AutoCommandState.GYRO_DRIVE;
                break;

    }
    */
}
