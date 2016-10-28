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
    DcMotor elevatorMotor;

    final int CATAPULT_POWER = 1;
    final int TARGET_POS = 3 * 1120; // Three rotations

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

        // Activates intake when B button is pressed
        if(gamepad1.b){
            intakeMotor.setPower(1);
        } else{
            intakeMotor.setPower(0);
        }

        // FIXME - Fix this portion of the code  //
        /*if(gamepad1.a == true){
         	int currentPos = 0;

            rightBackMotor.setTargetPosition(TARGET_POS);
            leftBackMotor.setTargetPosition(TARGET_POS);
            rightFrontMotor.setTargetPosition(TARGET_POS);
            leftFrontMotor.setTargetPosition(TARGET_POS);

            while(currentPos < TARGET_POS) {
                catapultMotor.setPower(CATAPULT_POWER);
                currentPos = catapultMotor.getCurrentPosition();
            }
        }
        catapultMotor.setPower(0); */
        //										//

        if(gamepad1.dpad_down){
            elevatorMotor.setPower(1);
        }
        else if(gamepad1.dpad_up){
            elevatorMotor.setPower(-1);
        }
        else {
        	elevatorMotor.setPower(0);
        }

        telemetry.addData("Left Front Motor Power", leftFrontMotor.getPowerFloat());
        telemetry.update();

    }

}

