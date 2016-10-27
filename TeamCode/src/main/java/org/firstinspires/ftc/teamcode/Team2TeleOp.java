package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Teacher on 9/28/2016.
 */

@TeleOp(name ="Team2: TeleOp", group ="Team2")
//@Disabled
public class Team2TeleOp extends OpMode {

    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor leftBackMotor;
    DcMotor intakeMotor;
    DcMotor catapultMotor;
    DcMotor elevatorMotor;

    int currentPos = 0;
    int catapultStrength;
    int motorRotations;

    @Override
    public void init() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        rightBackMotor = hardwareMap.dcMotor.get("rightRear");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        catapultMotor = hardwareMap.dcMotor.get("catapult");
        elevatorMotor = hardwareMap.dcMotor.get("elevator");

        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        gamepad1.setJoystickDeadzone((float) .1);
}

    @Override
    public void loop() {

        leftFrontMotor.setPower(gamepad1.left_stick_y);
        rightBackMotor.setPower(gamepad1.right_stick_y);
        rightFrontMotor.setPower(gamepad1.right_stick_y);
        leftBackMotor.setPower(gamepad1.left_stick_y);

        //activates intake when b button is pressed

        if(gamepad1.b == true){
            intakeMotor.setPower(1);
        }
        else{
            intakeMotor.setPower(0);
        }

        catapultStrength = 1;
        motorRotations = 1120*3;

        if(gamepad1.a == true){

            rightBackMotor.setTargetPosition(motorRotations);
            leftBackMotor.setTargetPosition(motorRotations);
            rightFrontMotor.setTargetPosition(motorRotations);
            leftFrontMotor.setTargetPosition(motorRotations);

            while(currentPos < motorRotations) {
                catapultMotor.setPower(catapultStrength);
                currentPos = catapultMotor.getCurrentPosition();
            }
        }
        else{
            catapultMotor.setPower(0);
        }

        if(gamepad1.dpad_down == true){
            elevatorMotor.setPower(1);
        }
        else if(gamepad1.dpad_up == true){
            elevatorMotor.setPower(-1);
        }

        catapultMotor.setPower(0);

        telemetry.addData("Left Front Motor Power", leftFrontMotor.getPowerFloat());
        telemetry.update();

    }

}

