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

	
	// Constants //
    final int SHOOTER_ROTATION 	  = 765;
    final double LEFT_SERVO_MIN   = 0.132;
    final double RIGHT_SERVO_MIN  = 0;
    final double LEFT_SERVO_HOME  = 0.74;
    final double RIGHT_SERVO_HOME = 0.55;

    // Member variables //
    int shooterOffset;

    boolean leftBeaconButton = false;
    boolean rightBeaconButton = false;
    
    // Hardware declarations //
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

        leftBeaconServo.scaleRange(LEFT_SERVO_MIN, LEFT_SERVO_HOME);
        rightBeaconServo.scaleRange(RIGHT_SERVO_MIN, RIGHT_SERVO_HOME);

        leftBeaconServo.setPosition(1);
        rightBeaconServo.setPosition(1);
    }

    @Override
    public void loop() {
        leftFrontMotor.setPower(gamepad1.left_stick_y);
        leftBackMotor.setPower(gamepad1.left_stick_y);
        rightFrontMotor.setPower(gamepad1.right_stick_y);
        rightBackMotor.setPower(gamepad1.right_stick_y);

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
            while(getShooterPosition() < SHOOTER_ROTATION) {
                shooterMotor.setPower(1);
            }
            shooterMotor.setPower(0);
            shooterOffset = shooterMotor.getCurrentPosition();
        }
        if(gamepad1.left_bumper) {
            shooterMotor.setPower(1);
        } else {
            shooterMotor.setPower(0);
        }

        if(gamepad1.a && !rightBeaconButton) {
            rightBeaconServo.setPosition((rightBeaconServo.getPosition() < 0.2) ? 1 : 0);
            rightBeaconButton = true;
        } else if(!gamepad1.a) rightBeaconButton = false;

        if(gamepad1.x && !leftBeaconButton) {
            leftBeaconServo.setPosition((leftBeaconServo.getPosition() < 0.2) ? 1 : 0);
            leftBeaconButton = true;
        } else if(!gamepad1.x) leftBeaconButton = false;
    }

    private int getShooterPosition() {
        return shooterMotor.getCurrentPosition() - shooterOffset;
    }
}

