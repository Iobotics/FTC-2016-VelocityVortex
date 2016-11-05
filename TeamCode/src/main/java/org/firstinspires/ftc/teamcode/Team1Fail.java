package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name = "Team 1 Fail: Autonomous", group = "Team 1")
//@Disabled
public class Team1Fail extends OpMode {

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

    }
    @Override
    public void loop() {
        frontLeftMotor.setPower(1.0);
        frontRightMotor.setPower(1.0);
        backLeftMotor.setPower(1.0);
        backRightMotor.setPower(1.0);

        try {
            Thread.sleep(1300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        for(double r =1;r > 0 ; r-=.1){
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            frontLeftMotor.setPower(r);
            frontRightMotor.setPower(r);
            backLeftMotor.setPower(r);
            backRightMotor.setPower(r);
        }
        catapultMotor.setPower(.3);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    this.requestOpModeStop();
    }
    @Override
    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}