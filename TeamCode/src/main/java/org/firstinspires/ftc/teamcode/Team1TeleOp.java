package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    final int FLYWHEEL_POWER = 1;
    final int TARGET_POS = 3 * 1120; // Three rotations

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

        // Tank drive
        leftFrontMotor.setPower(gamepad1.left_stick_y);
        leftBackMotor.setPower(gamepad1.left_stick_y);
        rightFrontMotor.setPower(gamepad1.right_stick_y);
        rightBackMotor.setPower(gamepad1.right_stick_y);

        // Intake motor is controlled through D-pad up and down
        if(gamepad1.dpad_down){
            intakeMotor.setPower(1);
        }
        else if(gamepad1.dpad_up){
            intakeMotor.setPower(-1);
        }
        else{
            intakeMotor.setPower(0);
        }

        // FIXME - Fix this portion of the code  //
        /* if(gamepad1.a){
         	int currentPos = 0;
         	
            rightBackMotor.setTargetPosition(TARGET_POS);
            leftBackMotor.setTargetPosition(TARGET_POS);
            rightFrontMotor.setTargetPosition(TARGET_POS);
            leftFrontMotor.setTargetPosition(TARGET_POS);

            while(currentPos < TARGET_POS) {
                catapultMotor.setPower(FLYWHEEL_POWER);
                currentPos = catapultMotor.getCurrentPosition();
            }
        }
        catapultMotor.setPower(0); */
        //										//

        telemetry.addData("Left Front Motor Power", leftFrontMotor.getPowerFloat());
        telemetry.update();
    }
}
