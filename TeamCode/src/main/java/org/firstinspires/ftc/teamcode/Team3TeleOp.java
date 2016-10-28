package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Teacher on 9/28/2016.
 */

@TeleOp(name ="Team3: TeleOp", group ="Team3")
//@Disabled
public class Team3TeleOp extends OpMode {

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
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBeaconServo.setPosition(.5);
        rightBeaconServo.setPosition(.5);

        gamepad1.setJoystickDeadzone((float) .1);
}

    @Override
    public void loop() {

        leftFrontMotor.setPower(gamepad1.left_stick_y);
        rightBackMotor.setPower(gamepad1.right_stick_y);
        rightFrontMotor.setPower(gamepad1.right_stick_y);
        leftBackMotor.setPower(gamepad1.left_stick_y);

        //activates intake when right trigger is pressed

        if(gamepad1.right_trigger > 0){
            intakeMotor.setPower(gamepad1.right_trigger);
        }
        else if(gamepad1.right_bumper == true){
            intakeMotor.setPower(-1);
        }
        else{
            intakeMotor.setPower(0);
        }

        //left trigger to use shooter
        if(gamepad1.left_trigger >0){
            shooterMotor.setPower(gamepad1.left_trigger);
            telemetry.addData("encoder position", shooterMotor.getTargetPosition());
            telemetry.update();
        }
        else{
            shooterMotor.setPower(0);
        }

        //a for right servo; x for left servo;
        if(gamepad1.a == true && rightBeaconServo.getPosition() == .5){
            rightBeaconServo.setPosition(.5);
        }
        else if(gamepad1.x == true && leftBeaconServo.getPosition() == .5){
            leftBeaconServo.setPosition(0);
        }
        else if(gamepad1.x == true && leftBeaconServo.getPosition() == 0){
            leftBeaconServo.setPosition(.5);
        }
        else if(gamepad1.a == true && rightBeaconServo.getPosition() == 1) {
            leftBeaconServo.setPosition(0);
        }
    }
}

