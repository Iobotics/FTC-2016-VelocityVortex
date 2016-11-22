package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Darren Kam on 10/28/2016.
 */
@TeleOp(name = "Team 3: Calibration", group = "Team 3")
//@Disabled
public class Team3Calibration extends OpMode {

	// Constants //
    final int LED_PORT            = 5;
    final int SHOOTER_ROTATION 	  = 730;
    final double LEFT_SERVO_MIN	  = 0.132;
    final double RIGHT_SERVO_MIN  = 0;
    final double LEFT_SERVO_HOME  = 0.74;
    final double RIGHT_SERVO_HOME = 0.55;

    final double REGULATOR_SERVO_MIN  = 0;
    final double REGULATOR_SERVO_HOME = 0.7;

    final double LIGHT_THRESHOLD = 0.24;

    // Member variables //
    int _shooterOffset;
    double _lightOffset;

    boolean _ledState = false;
    boolean _leftBeaconButton = false;
    boolean _rightBeaconButton = false;

    // Hardware declarations //
    DcMotor _rightFrontMotor;
    DcMotor _leftFrontMotor;
    DcMotor _rightBackMotor;
    DcMotor _leftBackMotor;

    DcMotor _intakeMotor;
    DcMotor _shooterMotor;

    Servo _leftBeaconServo;
    Servo _rightBeaconServo;

    Servo _regulatorServo;

    ColorSensor _sensorRGB;
    DeviceInterfaceModule _cdim;

    ModernRoboticsI2cGyro _gyro;

    LightSensor _lightSensor;

    @Override
    public void init() {
        _leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        _leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        _rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        _rightBackMotor = hardwareMap.dcMotor.get("rightRear");

        _rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        _rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        _intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        _shooterMotor = hardwareMap.dcMotor.get("shooter");

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
        _cdim.setDigitalChannelMode(LED_PORT, DigitalChannelController.Mode.OUTPUT);
        _cdim.setDigitalChannelState(LED_PORT, _ledState);

        _gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        _gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        _gyro.calibrate();

        _lightSensor = hardwareMap.lightSensor.get("light");
        _lightSensor.enableLed(true);

        _lightOffset = _lightSensor.getLightDetected();

        gamepad1.setJoystickDeadzone((float) 0.05);
    }
    
    @Override
    public void init_loop() {
    	telemetry.addData("Gyro", _gyro.isCalibrating() ? "Calibrating" : _gyro.getHeading());
    	telemetry.update();
    }

    @Override
    public void loop() {
        _leftFrontMotor.setPower(gamepad1.left_stick_y);
        _leftBackMotor.setPower(gamepad1.left_stick_y);
        _rightFrontMotor.setPower(gamepad1.right_stick_y);
        _rightBackMotor.setPower(gamepad1.right_stick_y);

        if(gamepad1.right_trigger > 0) {
            _intakeMotor.setPower(1);
        }
        else if(gamepad1.right_bumper) {
            _intakeMotor.setPower(-1);
        }
        else {
            _intakeMotor.setPower(0);
        }

        if(gamepad1.left_trigger > 0) {
            while(getShooterPosition() < SHOOTER_ROTATION) {
                _shooterMotor.setPower(1);
            }
            _shooterMotor.setPower(0);
            _shooterOffset = _shooterMotor.getCurrentPosition();
        }
        else if(gamepad1.left_bumper) {
            _shooterMotor.setPower(1);
        }
        else {
            _shooterMotor.setPower(0);
        }

        if(gamepad1.a && !_rightBeaconButton) {
            _rightBeaconServo.setPosition((_rightBeaconServo.getPosition() < 0.2) ? 1 : 0);
            _rightBeaconButton = true;
        } else if(!gamepad1.a) _rightBeaconButton = false;

        if(gamepad1.x && !_leftBeaconButton) {
            _leftBeaconServo.setPosition((_leftBeaconServo.getPosition() < 0.2) ? 1 : 0);
            _leftBeaconButton = true;
        } else if(!gamepad1.x) _leftBeaconButton = false;

        if(gamepad1.a) {
            _lightOffset = _lightSensor.getLightDetected();
        }

        telemetry.addData("Light Sensor", _lightSensor.getLightDetected() - _lightOffset);
        telemetry.addData("Line detected", (_lightSensor.getLightDetected() - _lightOffset >= LIGHT_THRESHOLD));
        telemetry.addData("Alpha", _sensorRGB.alpha());
        telemetry.addData("Red", _sensorRGB.red());
        telemetry.addData("Green", _sensorRGB.green());
        telemetry.addData("Blue", _sensorRGB.blue());
        telemetry.addData("Shooter pos", _shooterMotor.getCurrentPosition() - _shooterOffset);
        telemetry.addData("Regulator", _regulatorServo.getPosition());
        telemetry.addData("Intake", _intakeMotor.getCurrentPosition());
        telemetry.addData("Gyro", _gyro.getHeading());
        telemetry.update();
    }

    private int getShooterPosition() {
        return _shooterMotor.getCurrentPosition() - _shooterOffset;
    }
}
