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

    final int SHOOTER_TICKS 	  = 1120; // TODO - Calibrate value
    final double LEFT_SERVO_HOME  = 0.45;
    final double RIGHT_SERVO_HOME = 0.55;

    int shooterOffset;
    boolean leftBeaconButton;
    boolean rightBeaconButton;

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

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterOffset = shooterMotor.getCurrentPosition();

        leftBeaconServo.scaleRange(Servo.MIN_POSITION, LEFT_SERVO_HOME);
        rightBeaconServo.scaleRange(Servo.MIN_POSITION, RIGHT_SERVO_HOME);
        
        leftBeaconServo.setPosition(1);
        rightBeaconServo.setPosition(1);
        
        leftBeaconButton = true;
        rightBeaconButton = true;

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
            intakeMotor.setPower(1);
        }
        else if(gamepad1.right_bumper) {
            intakeMotor.setPower(-1);
        }
        else {
            intakeMotor.setPower(0);
        }

        // Left trigger to use shooter
        if(gamepad1.left_trigger > 0) {
            while((shooterMotor.getCurrentPosition() - shooterOffset) < SHOOTER_TICKS) {
                shooterMotor.setPower(1);
            }
            shooterMotor.setPower(0);
            shooterOffset = shooterMotor.getCurrentPosition();
        }

        // A for right servo; X for left servo
        if(gamepad1.a) {
            double position = rightBeaconButton ? 1 : 0;
            rightBeaconButton = !rightBeaconButton;
            rightBeaconServo.setPosition(position);
        }
        if(gamepad1.x) {
            double position = leftBeaconButton ? 1 : 0;
            leftBeaconButton = !leftBeaconButton;
            leftBeaconServo.setPosition(position);
        }
    }
}

