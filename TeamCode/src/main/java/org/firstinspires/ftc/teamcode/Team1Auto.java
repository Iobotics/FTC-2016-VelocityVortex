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

    int target;
    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");
        
        catapultMotor = hardwareMap.dcMotor.get("catapult");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        
        catapultMotor.setDirection(DcMotor.Direction.REVERSE);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void loop() {
        frontLeftMotor.setTargetPosition(targetPosition(12));
        frontRightMotor.setTargetPosition(targetPosition(12));
        backLeftMotor.setTargetPosition(targetPosition(12));
        backRightMotor.setTargetPosition(targetPosition(12));

        while(frontLeftMotor.getCurrentPosition() < targetPosition(12)) {
            frontLeftMotor.setPower(0.3);
            frontRightMotor.setPower(0.3);
            backLeftMotor.setPower(0.3);
            backRightMotor.setPower(0.3);
        }
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);

        // TODO - Optimize code
        catapultMotor.setTargetPosition(targetPosition((90.76/0.0393701)* Math.PI/2)); // 90.76 mm

        while (catapultMotor.getCurrentPosition() >= 0) {
            catapultMotor.setPower(0.2);
        }
        catapultMotor.setPower(0);

        // TODO - Optimize code
        target = targetPosition(24 * Math.PI/4);

        while(frontLeftMotor.getCurrentPosition() <target ) {
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

        while(frontLeftMotor.getCurrentPosition() >= targetPosition(12)) {
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
