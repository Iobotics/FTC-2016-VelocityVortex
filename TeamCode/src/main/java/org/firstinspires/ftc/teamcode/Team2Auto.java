package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name="Team2: Autonomous", group="Team2")
//@Disabled
public class Team2Auto extends OpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor intakeMotor;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        long initialTime = System.currentTimeMillis();

        //runs intake and shooter on start
        while (System.currentTimeMillis() - initialTime < 5000){
            intakeMotor.setPower(1.0);
        }
        intakeMotor.setPower(0.0);
    }

    @Override
    public void loop() {

        moveForward(110, 1.0);

    }
    private int distance(double distanceToGo){
        //a method to calculate the number of ticks to move the robot a number of inches
        final int diameter = 15;
        //final diameter of the wheels's
        double circumference = diameter * Math.PI;
        double ticks = 1120;
        double ticksPerCentimeter = ticks / circumference;
        double distanceInTicks = ticksPerCentimeter * distanceToGo;
        return (int)distanceInTicks;
    }

    private int rotation (int distanceToTurn){
        //method to calculate amount of ticks to move the robot to rotate it
        int chassisDiameter = 50;
        double chassisCircumference = chassisDiameter * Math.PI;
        double centimetersPerDegree = chassisCircumference / 360;
        double DistanceToTurnInTicks = centimetersPerDegree * distanceToTurn;
        return (int)DistanceToTurnInTicks;
    }

    private void moveForward(int targetDistance, double targetPower){
        //method that takes target distance in centimeters and power and moves forward accordingly
        int currentPosition = 0;
        int distanceToTravel = distance(targetDistance);

        frontLeftMotor.setTargetPosition(distanceToTravel);

        while (frontLeftMotor.getTargetPosition() > currentPosition){
            frontLeftMotor.setPower(targetPower);
            frontRightMotor.setPower(targetPower);
            backLeftMotor.setPower(targetPower);
            backRightMotor.setPower(targetPower);
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
