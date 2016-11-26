package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Darren Kam on 10/28/2016.
 */
@TeleOp(name = "Team 3: Calibration", group = "Team 3")
//@Disabled
public class Team3Calibration extends OpMode {

	Team3Robot _robot = new Team3Robot();

    boolean _leftBeaconButton = false;
    boolean _rightBeaconButton = false;
    
    boolean _liftButton = false;
    
    @Override
    public void init() {
    	_robot.init(hardwareMap, telemetry);
    }
    
    @Override
    public void init_loop() {
    	telemetry.addData("Gyro", _robot._gyro.isCalibrating() ? "Calibrating" : _robot._gyro.getHeading());
    	telemetry.update();
    }

    @Override
    public void loop() {
    	_robot.setPower(gamepad1.left_stick_y, gamepad1.right_stick_y);

        // Activates intake when right trigger is pressed
        if(gamepad1.right_trigger > 0) {
            _robot._intakeMotor.setPower(1);
        }
        else if(gamepad1.right_bumper) {
        	_robot._intakeMotor.setPower(-1);
        }
        else {
        	_robot._intakeMotor.setPower(0);
        }

        // Left trigger to use shooter
        if(gamepad1.left_trigger > 0) {
            long startTime = System.currentTimeMillis();
            while(System.currentTimeMillis() - startTime < Team3Robot.REGULATOR_TIME) {
            	_robot._regulatorServo.setPosition(0);
            }
            _robot._regulatorServo.setPosition(1);
            while(_robot.getShooterPosition() < Team3Robot.SHOOTER_ROTATION) {
            	_robot._shooterMotor.setPower(1);
            }
            _robot._shooterMotor.setPower(0);
            _robot._shooterOffset = _robot._shooterMotor.getCurrentPosition();
        }
        if(gamepad1.left_bumper) {
        	_robot._shooterMotor.setPower(0.6);
        } else {
        	_robot._shooterMotor.setPower(0);
        }

        if(gamepad1.a && !_leftBeaconButton) {
        	_robot._leftBeaconServo.setPosition((_robot._leftBeaconServo.getPosition() < 0.2) ? 1 : 0);
            _leftBeaconButton = true;
        } else if(!gamepad1.a) _leftBeaconButton = false;

        if(gamepad1.b && !_rightBeaconButton) {
        	_robot._rightBeaconServo.setPosition((_robot._rightBeaconServo.getPosition() < 0.2) ? 1 : 0);
            _rightBeaconButton = true;
        } else if(!gamepad1.b) _rightBeaconButton = false;
        
        if(gamepad1.x && !_liftButton) {
        	_robot.setLiftPosition((_robot.getLiftPosition() > 0) ? Team3Robot.LIFT_HOME : Team3Robot.LIFT_UP);
            _liftButton = true;
        } else if(!gamepad1.x) _liftButton = false;

        telemetry.addData("Light Sensor", _robot.getLight());
        telemetry.addData("Line detected", (_robot.getLight() >= Team3Robot.LIGHT_THRESHOLD));
        telemetry.addData("Red", _robot._sensorRGB.red());
        telemetry.addData("Green", _robot._sensorRGB.green());
        telemetry.addData("Blue", _robot._sensorRGB.blue());
        telemetry.addData("Shooter pos", _robot.getShooterPosition());
        telemetry.addData("Regulator", _robot._regulatorServo.getPosition());
        telemetry.addData("Intake", _robot.getIntakePosition());
        telemetry.addData("Gyro", _robot.getGyro());
        telemetry.update();
    }
}
