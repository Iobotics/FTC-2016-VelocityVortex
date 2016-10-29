package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name = "Team 1: Autonomous", group = "Team 1")
//@Disabled
public class Team1Auto extends OpMode {
	
	final int    ENCODER_TICKS_PER_REV = 1120; // Neverest 40
    final int    WHEEL_DIAMETER        = 6;
    final double INCHES_PER_TICK       = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV; // INCHES / REV
	
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    
    DcMotor catapultMotor;

    int leftOffset;
    int catapultOffset;
    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");
        
        catapultMotor = hardwareMap.dcMotor.get("catapult");

       leftOffset = frontLeftMotor.getCurrentPosition();
        catapultOffset = catapultMotor.getCurrentPosition();

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        catapultMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        
        catapultMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        frontLeftMotor.setTargetPosition(targetPosition(59-(22+17.5)));
        frontRightMotor.setTargetPosition(targetPosition(59-(22+17.5)));
        backLeftMotor.setTargetPosition(targetPosition(59-(22+17.5)));
        backRightMotor.setTargetPosition(targetPosition(59-(22+17.5)));

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(frontLeftMotor.getCurrentPosition() - leftOffset <= frontLeftMotor.getTargetPosition()) {
            frontLeftMotor.setPower(0.3);
            frontRightMotor.setPower(0.3);
            backLeftMotor.setPower(0.3);
            backRightMotor.setPower(0.3);

            telemetry.addData("Current Position", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Target Position", frontLeftMotor.getTargetPosition());
            telemetry.update();
        }
        telemetry.addData("Is Out", 0);
        telemetry.update();

        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);

        catapultMotor.setTargetPosition(targetPosition(1120 * 3)); // 90.76 mm is the diameter of the catapult shell
// I put 3x the circumfrence as the distance of the catapult to move launch the ball

        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (catapultMotor.getCurrentPosition() - catapultOffset <= catapultMotor.getTargetPosition()) {
            catapultMotor.setPower(0.2);
        }
        catapultMotor.setPower(0);

        //will turn 1/4 of the robot in a circular motion
        frontLeftMotor.setTargetPosition(targetPosition(24 * Math.PI/4));
        frontRightMotor.setTargetPosition(targetPosition(24 * Math.PI/4));
        backLeftMotor.setTargetPosition(targetPosition(24 * Math.PI/4));
        backRightMotor.setTargetPosition(targetPosition(24 * Math.PI/4));

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(frontLeftMotor.getCurrentPosition() <= frontLeftMotor.getTargetPosition()) {
            frontLeftMotor.setPower(0.3);
            frontRightMotor.setPower(-0.3);
            backLeftMotor.setPower(0.3);
            backRightMotor.setPower(-0.3);
        }
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);

        frontLeftMotor.setTargetPosition(targetPosition(12));
        frontRightMotor.setTargetPosition(targetPosition(12));
        backLeftMotor.setTargetPosition(targetPosition(12));
        backRightMotor.setTargetPosition(targetPosition(12));

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(frontLeftMotor.getCurrentPosition() <= frontLeftMotor.getTargetPosition()) {
            frontLeftMotor.setPower(0.3);
            frontRightMotor.setPower(0.3);
            backLeftMotor.setPower(0.3);
            backRightMotor.setPower(0.3);
        }
        stop();
    }

    @Override
    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private int targetPosition(double distance){
        double distanceInTicks = distance / INCHES_PER_TICK;
        return (int) distanceInTicks;
    }
}
