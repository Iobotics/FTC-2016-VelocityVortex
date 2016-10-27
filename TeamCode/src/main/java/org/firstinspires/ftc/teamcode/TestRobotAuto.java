package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name="TestRobot: Autonomous", group="TestRobot")
//@Disabled
public class TestRobotAuto extends OpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        long initial_time = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        int currentPosition = 0;
        int distanceToTravel = distance(23.0);

        frontLeftMotor.setTargetPosition(distanceToTravel);

        while (frontLeftMotor.getTargetPosition() > 0){
            frontLeftMotor.setPower(1.0);
            frontRightMotor.setPower(1.0);
            backLeftMotor.setPower(1.0);
            backRightMotor.setPower(1.0);
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public int distance(double distanceToGo){
        int diameter = 15;
        double circumference = diameter * Math.PI;
        double ticks = 1120;
        double ticksPerCentimeter = ticks / circumference;
        double distanceInTicks = ticksPerCentimeter * distanceToGo;
        return (int)distanceInTicks;
    }
    public int rotation (int distanceToTurn){
        int chassisDiameter = 10;
        double chassisCircumference = chassisDiameter * Math.PI;
        double centimetersPerDegree = chassisCircumference / 360;
        double DistanceToTurnInTicks = centimetersPerDegree * distanceToTurn;
        return (int)DistanceToTurnInTicks;
    }
}
