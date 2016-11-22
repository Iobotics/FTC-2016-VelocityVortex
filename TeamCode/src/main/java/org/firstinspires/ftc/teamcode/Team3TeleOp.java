package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Darren Kam on 9/28/2016.
 */
@TeleOp(name = "Team 3: TeleOp", group = "Team 3")
//@Disabled
public class Team3TeleOp extends Team3Base {

    boolean leftBeaconButton = false;
    boolean rightBeaconButton = false;

    @Override
    public void robotInit() {
        gamepad1.setJoystickDeadzone((float) 0.05);
    }

    @Override
    public void robotLoop() {
        _leftFrontMotor.setPower(gamepad1.left_stick_y);
        _leftBackMotor.setPower(gamepad1.left_stick_y);
        _rightFrontMotor.setPower(gamepad1.right_stick_y);
        _rightBackMotor.setPower(gamepad1.right_stick_y);

        // Activates intake when right trigger is pressed
        if(gamepad1.right_trigger > 0) {
            _intakeMotor.setPower(1.0);
        }
        else if(gamepad1.right_bumper) {
            _intakeMotor.setPower(-1.0);
        }
        else {
            _intakeMotor.setPower(0.0);
        }

        // Left trigger to use shooter
        if(gamepad1.left_trigger > 0) {
            long startTime = System.currentTimeMillis();
            while(System.currentTimeMillis() - startTime < REGULATOR_TIME) {
                _regulatorServo.setPosition(0);
            }
            _regulatorServo.setPosition(1);
            while(getShooterPosition() < SHOOTER_ROTATION) {
                _shooterMotor.setPower(1.0);
            }
            _shooterMotor.setPower(0);
            _shooterOffset = _shooterMotor.getCurrentPosition();
        }
        if(gamepad1.left_bumper) {
            _shooterMotor.setPower(0.6);
        } else {
            _shooterMotor.setPower(0.0);
        }

        if(gamepad1.a && !rightBeaconButton) {
            _rightBeaconServo.setPosition((_rightBeaconServo.getPosition() < 0.2) ? 1 : 0);
            rightBeaconButton = true;
        } else if(!gamepad1.a) rightBeaconButton = false;

        if(gamepad1.x && !leftBeaconButton) {
            _leftBeaconServo.setPosition((_leftBeaconServo.getPosition() < 0.2) ? 1 : 0);
            leftBeaconButton = true;
        } else if(!gamepad1.x) leftBeaconButton = false;
    }
}

