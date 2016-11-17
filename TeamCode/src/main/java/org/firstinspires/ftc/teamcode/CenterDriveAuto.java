package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name ="CenterDriveAuto: Autonomous", group ="CenterDriveAuto")
@Disabled
public class CenterDriveAuto extends OpMode {

    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor leftBackMotor;
    DcMotor shooterMotor;
    //Servo beaconServo;

    //ColorSensor beaconColorSensor;

    //int redValue;
    //int blueValue;
    //int greenValue;


    @Override
    public void init() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        rightBackMotor = hardwareMap.dcMotor.get("rightRear");
        shooterMotor = hardwareMap.dcMotor.get("");//TODO - name hardware
        //beaconServo = hardwareMap.servo.get("");  //TODO - name hardware

        //beaconColorSensor = hardwareMap.colorSensor.get("color");

        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        //begin autonomous
        //shoot particle
        shooterMotor.setPower(1);

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        shooterMotor.setPower(0);

        int distanceToTravel = distance(110);

        leftBackMotor.setTargetPosition(distanceToTravel);
        leftFrontMotor.setTargetPosition(distanceToTravel);
        rightBackMotor.setTargetPosition(distanceToTravel);
        rightFrontMotor.setTargetPosition(distanceToTravel);

        int currentPos = 0;

        while(currentPos < distanceToTravel){
            leftFrontMotor.setPower(0.7);
            leftBackMotor.setPower(0.7);
            rightFrontMotor.setPower(0.7);
            rightBackMotor.setPower(0.7);
            currentPos=rightBackMotor.getCurrentPosition();
            telemetry.addData("Current Position", currentPos);
        }

        this.requestOpModeStop();

        distanceToTravel = distance(110);

        leftBackMotor.setTargetPosition(distanceToTravel);
        leftFrontMotor.setTargetPosition(distanceToTravel);
        rightBackMotor.setTargetPosition(distanceToTravel);
        rightFrontMotor.setTargetPosition(distanceToTravel);

        currentPos = 0;

        while(currentPos < distanceToTravel){
            leftFrontMotor.setPower(0.7);
            leftBackMotor.setPower(0.7);
            rightFrontMotor.setPower(0.7);
            rightBackMotor.setPower(0.7);
            currentPos=rightBackMotor.getCurrentPosition();
            telemetry.addData("Current Position", currentPos);
        }

    }

    @Override
    public void stop() {
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    public int distance(double distanceToGo){ //distance in centimeters
        double diameter = 15; //also in centimeters  **set to 10.16 for all teams; **
        double circumference = diameter*java.lang.Math.PI;
        double ticks = 1120;
        double ticksPerCentimeter= ticks/circumference;
        double distanceInTicks= ticksPerCentimeter*distanceToGo;
        return (int)distanceInTicks;
    }
}
