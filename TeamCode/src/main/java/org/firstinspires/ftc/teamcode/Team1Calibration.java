package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name = "Team 1 Calibration: Teleop", group = "Team 1")
//@Disabled
public class Team1Calibration extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor catapultMotor;
    DcMotor intakeMotor;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");
        catapultMotor = hardwareMap.dcMotor.get("catapult");

    }
    @Override
    public void loop() {
        frontLeftMotor.setPower(gamepad1.left_stick_y);
        backLeftMotor.setPower(gamepad1.left_stick_y);
        frontRightMotor.setPower(gamepad1.right_stick_y);
        backRightMotor.setPower(gamepad1.right_stick_y);

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

        if(gamepad1.right_trigger>0){

        }
            /*
             while((catapultMotor.getCurrentPosition() - catapultOffset) < CATAPULT_TICKS) {
                 catapultMotor.setPower(1);
                 leftFrontMotor.setPower(gamepad1.left_stick_y);
                 leftBackMotor.setPower(gamepad1.left_stick_y);
                 rightFrontMotor.setPower(gamepad1.right_stick_y);
                 rightBackMotor.setPower(gamepad1.right_stick_y);
             }
             catapultMotor.setPower(0);
             catapultOffset = catapultMotor.getCurrentPosition();
        }*/
        if(gamepad1.right_trigger>0){
            /*
            time.reset();
            while(time.milliseconds()<1500){
                // Tank drive
                leftFrontMotor.setPower(gamepad1.left_stick_y);
                leftBackMotor.setPower(gamepad1.left_stick_y);
                rightFrontMotor.setPower(gamepad1.right_stick_y);
                rightBackMotor.setPower(gamepad1.right_stick_y);

                catapultMotor.setPower(1);
            }
            catapultMotor.setPower(0);
            */
            catapultMotor.setPower(1);
        }
        else{
            catapultMotor.setPower(0);
        }

        telemetry.addData("frontLeft Motor Position", frontLeftMotor.getCurrentPosition());
        telemetry.addData("backLeft Motor Position", backLeftMotor.getCurrentPosition());
        telemetry.addData("frontRight Motor Position", frontRightMotor.getCurrentPosition());
        telemetry.addData("backRight Motor Position", backRightMotor.getCurrentPosition());
        telemetry.addData("Catapult Motor Position", catapultMotor.getCurrentPosition());
        telemetry.addData("Intake Motor Position", intakeMotor.getCurrentPosition());
        telemetry.update();
    }
    @Override
    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}