package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Teacher on 9/28/2016.
 */

@TeleOp(name = "Team 1: TeleOp", group = "Team 1")
//@Disabled
public class Team1TeleOp extends OpMode {

    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor leftBackMotor;
    
    DcMotor intakeMotor;
    DcMotor catapultMotor;

    final int CATAPULT_TICKS = 3 * 1120; // Three rotations

    int catapultOffset;

    @Override
    public void init() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        rightBackMotor = hardwareMap.dcMotor.get("rightRear");

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        //intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        catapultMotor = hardwareMap.dcMotor.get("catapult");

        catapultMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        catapultOffset = -catapultMotor.getCurrentPosition();

        gamepad1.setJoystickDeadzone((float) 0.05);
}

    @Override
    public void loop() {

        // Tank drive
        leftFrontMotor.setPower(gamepad1.left_stick_y);
        leftBackMotor.setPower(gamepad1.left_stick_y);
        rightFrontMotor.setPower(gamepad1.right_stick_y);
        rightBackMotor.setPower(gamepad1.right_stick_y);

        // Intake motor is controlled through left bumper and right bumper
        if(gamepad1.left_bumper){
            intakeMotor.setPower(1);
        }
        else if(gamepad1.right_bumper){
            intakeMotor.setPower(-1);
        }
        else{
            intakeMotor.setPower(0);
        }

        if(gamepad1.x){
            catapultMotor.setPower(1);
        }
        else{
            catapultMotor.setPower(0);
        }

        if(gamepad1.right_trigger > 0) {
            while(catapultMotor.getCurrentPosition() - catapultOffset < CATAPULT_TICKS) {
                catapultMotor.setPower(1);
            }
            catapultMotor.setPower(0);
            catapultOffset = catapultMotor.getCurrentPosition();
        }
    }

}
