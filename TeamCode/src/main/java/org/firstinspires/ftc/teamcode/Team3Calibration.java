package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

	Team3Robot _robot = new Team3Robot();

    double speedMultiplier = 1;
    
    @Override
    public void init() {
    	_robot.init(hardwareMap, telemetry);
    }
    
    @Override
    public void init_loop() {
    	telemetry.addData("Gyro", _robot.isGyroCalibrating() ? "Calibrating" : _robot.getGyroHeading());
    	telemetry.update();
    }

    @Override
    public void loop() {
        // Basic tank drive
        _robot.setPower(gamepad1.left_stick_y * speedMultiplier, gamepad1.right_stick_y * speedMultiplier);

        // Left trigger to use shooter
        if(gamepad1.left_trigger > 0) {
            _robot.shootBall();
            _robot.resetShooterEncoder();
        }
        else if(gamepad1.left_bumper) {
            _robot.setShooterPower(0.6);
        }
        else {
            _robot.setShooterPower(0);
        }

        // Activates intake when right trigger is pressed
        if(gamepad1.right_trigger > 0) {
            _robot.setIntakePower(1);
        }
        else if(gamepad1.right_bumper) {
            _robot.setIntakePower(-1);
        }
        else {
            _robot.setIntakePower(0);
        }

        // Button A to toggle left beacon servo
        if(gamepad1.a && !_robot._leftBeaconButton) {
            _robot.setLeftServo((_robot.getLeftServo() < 0.2) ? 1 : 0);
            _robot._leftBeaconButton = true;
        } else if(!gamepad1.a) _robot._leftBeaconButton = false;

        // Button B to toggle right beacon servo
        if(gamepad1.b && !_robot._rightBeaconButton) {
            _robot.setRightServo((_robot.getRightServo() < 0.2) ? 1 : 0);
            _robot._rightBeaconButton = true;
        } else if(!gamepad1.b) _robot._rightBeaconButton = false;

        if(gamepad1.x) {
            _robot.setLiftPower(Team3Robot.LIFT_POWER);
        }
        else if(gamepad1.y) {
            _robot.setLiftPower(-Team3Robot.LIFT_POWER);
        }
        else {
            _robot.setLiftPower(0);
        }

        // Start button to toggle half speed
        if(gamepad1.start && !_robot._slowButton) {
            speedMultiplier = Team3Robot.HALF_SPEED;
            _robot._slowButton = true;
        } else if(!gamepad1.start) _robot._slowButton = false;

        telemetry.addData("Light Sensor", _robot.getLightReading());
        telemetry.addData("Line detected", (_robot.getLightReading() >= Team3Robot.LIGHT_THRESHOLD));
        telemetry.addData("Lift pos", _robot.getLiftPosition());
        telemetry.addData("Shooter pos", _robot.getShooterPosition());
        telemetry.addData("Gyro", _robot.getGyroHeading());
        telemetry.update();
    }
}
