package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Matthew Leong on 9/28/2016.
 * Edited by Darren Kam
 */
@TeleOp(name = "Team 3: TeleOp", group = "Team 3")
//@Disabled
public class Team3TeleOp extends OpMode {

	// Constants //
    final int SHOOTER_ROTATION 	  = 720;
    final double LEFT_SERVO_MIN   = 0.132;
    final double RIGHT_SERVO_MIN  = 0;
    final double LEFT_SERVO_HOME  = 0.74;
    final double RIGHT_SERVO_HOME = 0.55;

    final double REGULATOR_SERVO_MIN  = 0;
    final double REGULATOR_SERVO_HOME = 0.7;

    final long REGULATOR_TIME = 800;

    // Member variables //
    int _shooterOffset;

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

    @Override
    public void init() {
        _leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        _rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        _leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        _rightBackMotor = hardwareMap.dcMotor.get("rightRear");
        
        _intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        _shooterMotor = hardwareMap.dcMotor.get("shooter");
        
        _rightBeaconServo = hardwareMap.servo.get("rightBeacon");
        _leftBeaconServo = hardwareMap.servo.get("leftBeacon");

        _regulatorServo = hardwareMap.servo.get("regulator");

        _rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        _rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        
        _shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        _shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _shooterOffset = _shooterMotor.getCurrentPosition();

        _leftBeaconServo.scaleRange(LEFT_SERVO_MIN, LEFT_SERVO_HOME);
        _rightBeaconServo.scaleRange(RIGHT_SERVO_MIN, RIGHT_SERVO_HOME);

        _regulatorServo.scaleRange(REGULATOR_SERVO_MIN, REGULATOR_SERVO_HOME);

        _leftBeaconServo.setPosition(1);
        _rightBeaconServo.setPosition(1);

        _regulatorServo.setPosition(1);

        gamepad1.setJoystickDeadzone((float) 0.05);
    }

    @Override
    public void loop() {
        _leftFrontMotor.setPower(gamepad1.left_stick_y);
        _leftBackMotor.setPower(gamepad1.left_stick_y);
        _rightFrontMotor.setPower(gamepad1.right_stick_y);
        _rightBackMotor.setPower(gamepad1.right_stick_y);

        // Activates intake when right trigger is pressed
        if(gamepad1.right_trigger > 0) {
            _intakeMotor.setPower(1);
        }
        else if(gamepad1.right_bumper) {
            _intakeMotor.setPower(-1);
        }
        else {
            _intakeMotor.setPower(0);
        }

        // Left trigger to use shooter
        if(gamepad1.left_trigger > 0) {
            long startTime = System.currentTimeMillis();
            while(System.currentTimeMillis() - startTime < REGULATOR_TIME) {
                _regulatorServo.setPosition(0);
            }
            _regulatorServo.setPosition(1);
            while(getShooterPosition() < SHOOTER_ROTATION) {
                _shooterMotor.setPower(1);
            }
            _shooterMotor.setPower(0);
            _shooterOffset = _shooterMotor.getCurrentPosition();
        }
        if(gamepad1.left_bumper) {
            _shooterMotor.setPower(0.6);
        } else {
            _shooterMotor.setPower(0);
        }

        if(gamepad1.a && !_leftBeaconButton) {
            _leftBeaconServo.setPosition((_leftBeaconServo.getPosition() < 0.2) ? 1 : 0);
            _leftBeaconButton = true;
        } else if(!gamepad1.a) _leftBeaconButton = false;

        if(gamepad1.b && !_rightBeaconButton) {
            _rightBeaconServo.setPosition((_rightBeaconServo.getPosition() < 0.2) ? 1 : 0);
            _rightBeaconButton = true;
        } else if(!gamepad1.b) _rightBeaconButton = false;
    }

    private int getShooterPosition() {
        return _shooterMotor.getCurrentPosition() - _shooterOffset;
    }
}