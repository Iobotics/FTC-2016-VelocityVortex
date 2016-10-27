package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name="Team1: Autonomous", group="Team1")
//@Disabled
public class Team1Auto extends OpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor catapultMotor;

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
    }

    @Override
    public void loop() {

        frontLeftMotor.setTargetPosition(targetPosition(12,6));
        frontRightMotor.setTargetPosition(targetPosition(12,6));
        backLeftMotor.setTargetPosition(targetPosition(12,6));
        backRightMotor.setTargetPosition(targetPosition(12,6));

        while(0 <= frontLeftMotor.getCurrentPosition())
        {
            frontLeftMotor.setPower(0.3);
            frontRightMotor.setPower(0.3);
            backLeftMotor.setPower(0.3);
            backRightMotor.setPower(0.3);
        }
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);

        catapultMotor.setTargetPosition(targetPosition((90.76/0.0393701)* Math.PI/2,6)); //TODO- 90.76 mm

        while (0 <= catapultMotor.getCurrentPosition())
        {
            catapultMotor.setPower(.2);
        }
        catapultMotor.setPower(0);

        frontLeftMotor.setTargetPosition(targetPosition(24 * Math.PI/4,6));
        frontRightMotor.setTargetPosition(targetPosition(24 * Math.PI/4,6));
        backLeftMotor.setTargetPosition(targetPosition(24 * Math.PI/4,6));
        backRightMotor.setTargetPosition(targetPosition(24 * Math.PI/4,6));

        while(0 <= frontLeftMotor.getCurrentPosition())
        {
            frontLeftMotor.setPower(0.3);
            frontRightMotor.setPower(-0.3);
            backLeftMotor.setPower(0.3);
            backRightMotor.setPower(-0.3);
        }
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);

        frontLeftMotor.setTargetPosition(targetPosition(12,6));
        frontRightMotor.setTargetPosition(targetPosition(12,6));
        backLeftMotor.setTargetPosition(targetPosition(12,6));
        backRightMotor.setTargetPosition(targetPosition(12,6));

        while(0 <= frontLeftMotor.getCurrentPosition())
        {
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

    private int targetPosition(double distanceToGo, double wheelDiameter){
        double push;
        double circumfrence;
        circumfrence= wheelDiameter * Math.PI;
        push= distanceToGo / circumfrence;
        push= push * 1120;
        return (int) push;
    }
}
