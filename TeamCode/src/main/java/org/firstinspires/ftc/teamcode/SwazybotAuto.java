package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name ="Swazybot: Autonomous", group ="Swazybot")
@Disabled
public class SwazybotAuto extends OpMode {

    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor leftBackMotor;

    Servo beaconServo;

    ColorSensor beaconColorSensor;

    int redValue;
    int blueValue;
    int greenValue;


    @Override
    public void init() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        rightBackMotor = hardwareMap.dcMotor.get("rightRear");

        beaconServo = hardwareMap.servo.get("");  //TODO - name hardware

        beaconColorSensor = hardwareMap.colorSensor.get("color");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        while (redValue < 200 && blueValue < 200){
            leftFrontMotor.setPower(0.4);
            leftBackMotor.setPower(0.4);
            rightFrontMotor.setPower(0.4);
            rightBackMotor.setPower(0.4);
            redValue = beaconColorSensor.red();
            blueValue = beaconColorSensor.blue();
            greenValue = beaconColorSensor.green();
        }

        leftFrontMotor.setPower(0.2);
        leftBackMotor.setPower(0.2);
        rightFrontMotor.setPower(0.2);
        rightBackMotor.setPower(0.2);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        this.requestOpModeStop();
    }

    @Override
    public void stop() {
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
}
