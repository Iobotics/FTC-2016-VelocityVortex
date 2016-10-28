package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Teacher on 9/28/2016.
 */

@TeleOp(name ="Team1: TeleOp", group ="Team1")
//@Disabled
public class Team1TeleOp extends OpMode {

    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor leftBackMotor;
    DcMotor intakeMotor;
    DcMotor catapultMotor;

    int flyWheelStrength;
    int currentPos;
    int motorRotations;

    @Override
    public void init() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        rightBackMotor = hardwareMap.dcMotor.get("rightRear");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        catapultMotor = hardwareMap.dcMotor.get("catapult");

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        catapultMotor.setDirection(DcMotor.Direction.REVERSE);

        gamepad1.setJoystickDeadzone((float) .1);
}

    @Override
    public void loop() {

        //tank drive using joysticks

        leftFrontMotor.setPower(gamepad1.left_stick_y);
        rightBackMotor.setPower(gamepad1.right_stick_y);
        rightFrontMotor.setPower(gamepad1.right_stick_y);
        leftBackMotor.setPower(gamepad1.left_stick_y);

        //intake motor is controlled through dpad up and down

        if(gamepad1.dpad_down == true){
            intakeMotor.setPower(1);
        }
        else if(gamepad1.dpad_up == true){
            intakeMotor.setPower(-1);
        }
        else{
            intakeMotor.setPower(0);
        }


        flyWheelStrength = 1;
        motorRotations = 1120*3;

        if(gamepad1.a == true){

            rightBackMotor.setTargetPosition(motorRotations);
            leftBackMotor.setTargetPosition(motorRotations);
            rightFrontMotor.setTargetPosition(motorRotations);
            leftFrontMotor.setTargetPosition(motorRotations);

            while(currentPos < motorRotations) {
                catapultMotor.setPower(flyWheelStrength);
                currentPos = catapultMotor.getCurrentPosition();
            }
        }
        else{
            catapultMotor.setPower(0);
        }

        catapultMotor.setPower(0);

        telemetry.addData("Left Front Motor Power", leftFrontMotor.getPowerFloat());
        telemetry.update();
    }
}
