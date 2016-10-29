package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Teacher on 9/28/2016.
 */

@TeleOp(name = "Team 2: TeleOp", group = "Team 2")
//@Disabled
public class Team2TeleOp extends OpMode {

    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor leftBackMotor;
    
    DcMotor intakeMotor;
    DcMotor catapultMotor;

    final int CATAPULT_POWER = 1;
    final int TARGET_POS = 3 * 1120; // Three rotations

    int catapultOffset;

    @Override
    public void init() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        rightBackMotor = hardwareMap.dcMotor.get("rightRear");
        
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        catapultMotor = hardwareMap.dcMotor.get("catapult");

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        catapultMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        catapultOffset = catapultMotor.getCurrentPosition();

        gamepad1.setJoystickDeadzone((float) .1);
}

    @Override
    public void loop() {

        leftFrontMotor.setPower(gamepad1.left_stick_y);
        rightBackMotor.setPower(gamepad1.right_stick_y);
        rightFrontMotor.setPower(gamepad1.right_stick_y);
        leftBackMotor.setPower(gamepad1.left_stick_y);

        // Activates intake when B button is pressed
        if (gamepad1.b) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }

        if (gamepad1.a) {
            while ((catapultMotor.getCurrentPosition() - catapultOffset) < TARGET_POS) {
                catapultMotor.setPower(CATAPULT_POWER);
            }
        }
        catapultMotor.setPower(0);

    }
}

