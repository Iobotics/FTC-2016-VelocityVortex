package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Teacher on 9/28/2016.
 */

@TeleOp(name = "Team 3: TeleOp", group = "Team 3")
//@Disabled
public class Team3TeleOp extends OpMode {

    final int TARGET_POS = 1120; // TODO - Calibrate value

    int shooterOffset;

    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor leftBackMotor;
    
    DcMotor intakeMotor;
    DcMotor shooterMotor;
    
    Servo rightBeaconServo;
    Servo leftBeaconServo;

    @Override
    public void init() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        rightBackMotor = hardwareMap.dcMotor.get("rightRear");
        
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        shooterMotor = hardwareMap.dcMotor.get("shooter");
        
        rightBeaconServo = hardwareMap.servo.get("rightBeacon");
        leftBeaconServo = hardwareMap.servo.get("leftBeacon");

        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterOffset = shooterMotor.getCurrentPosition();

        leftBeaconServo.setPosition(.5);  // TODO - Calibrate value
        rightBeaconServo.setPosition(.5); // TODO - Calibrate value

        gamepad1.setJoystickDeadzone((float) .1);
    }

    @Override
    public void loop() {

        leftFrontMotor.setPower(gamepad1.left_stick_y);
        rightBackMotor.setPower(gamepad1.right_stick_y);
        rightFrontMotor.setPower(gamepad1.right_stick_y);
        leftBackMotor.setPower(gamepad1.left_stick_y);

        // Activates intake when right trigger is pressed
        if(gamepad1.right_trigger > 0) {
            intakeMotor.setPower(gamepad1.right_trigger);
        }
        else if(gamepad1.right_bumper) {
            intakeMotor.setPower(-1);
        }
        else {
            intakeMotor.setPower(0);
        }

        // Left trigger to use shooter
        if(gamepad1.left_trigger > 0) {
            while((shooterMotor.getCurrentPosition() - shooterOffset) < TARGET_POS) {
                shooterMotor.setPower(gamepad1.left_trigger);
                telemetry.addData("Shooter position", shooterMotor.getCurrentPosition());
                telemetry.update();
            }
            shooterMotor.setPower(0);
            shooterOffset = shooterMotor.getCurrentPosition();
        }

        // A for right servo; X for left servo
        boolean leftBeaconButton = true;
        boolean rightBeaconButton = true;
        if(gamepad1.a) {
            double position = (rightBeaconButton) ? 0.5 : 0;
            rightBeaconButton = !rightBeaconButton;
            rightBeaconServo.setPosition(position);
        }
        else if(gamepad1.x) {
            double position = (leftBeaconButton) ? 0.5 : 0;
            leftBeaconButton = !leftBeaconButton;
            leftBeaconServo.setPosition(position);
        }
    }
}

