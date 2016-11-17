package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Teacher on 9/28/2016.
 */

@TeleOp(name = "Team 1 Calibration: Teleop", group = "Team 1")
//@Disabled
public class Team1Calibration extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor catapultMotor;
    DcMotor intakeMotor;

  //  ModernRoboticsI2cGyro gyro;
    int frontLeftOffSet;
    int frontRightOffSet;
    int backLeftOffSet;
    int backRightOffSet;
    int catapultOffset;
    int intakeOffSet;


    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");
/*
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        gyro.calibrate();
*/
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        catapultMotor = hardwareMap.dcMotor.get("catapult");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        catapultMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        catapultOffset = catapultMotor.getCurrentPosition();

        gamepad1.setJoystickDeadzone((float) .1);

        frontLeftOffSet = frontLeftMotor.getCurrentPosition();
        frontRightOffSet = frontRightMotor.getCurrentPosition();
        backLeftOffSet = backLeftMotor.getCurrentPosition();
        backRightOffSet = backRightMotor.getCurrentPosition();
        catapultOffset = catapultMotor.getCurrentPosition();
        intakeOffSet = intakeMotor.getCurrentPosition();


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

        telemetry.addData("frontLeft Motor Position", frontLeftMotor.getCurrentPosition() - frontLeftOffSet);
        telemetry.addData("backLeft Motor Position", backLeftMotor.getCurrentPosition() - backLeftOffSet); //negative
        telemetry.addData("frontRight Motor Position", frontRightMotor.getCurrentPosition()- frontRightOffSet); //negative
        telemetry.addData("backRight Motor Position", backRightMotor.getCurrentPosition()- backRightOffSet); //negative
        telemetry.addData("Catapult Motor Position", catapultMotor.getCurrentPosition() - catapultOffset); //negative
        telemetry.addData("Intake Motor Position", intakeMotor.getCurrentPosition() - intakeOffSet); //negative
        //telemetry.addData("Gyro Sensor", gyro.getHeading());
        //telemetry.update();
    }
    @Override
    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}