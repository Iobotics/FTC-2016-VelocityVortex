package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Teacher on 9/28/2016.
 */

@TeleOp(name ="Team2: TeleOp", group ="Team2")
//@Disabled
public class Team2TeleOp extends OpMode {

    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor leftBackMotor;
    DcMotor intakeMotor;
    DcMotor rightFlyWheelMotor;
    DcMotor leftFlyWheelMotor;

    @Override
    public void init() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        rightBackMotor = hardwareMap.dcMotor.get("rightRear");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        leftFlyWheelMotor = hardwareMap.dcMotor.get("leftFlyWheel");
        rightFlyWheelMotor = hardwareMap.dcMotor.get("rightFlyWheel");

        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFlyWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        gamepad1.setJoystickDeadzone((float) .1);
}

    @Override
    public void loop() {

        leftFrontMotor.setPower(gamepad1.left_stick_y);
        rightBackMotor.setPower(gamepad1.right_stick_y);
        rightFrontMotor.setPower(gamepad1.right_stick_y);
        leftBackMotor.setPower(gamepad1.left_stick_y);

        //activates intake when b button is pressed

        if(gamepad1.b == true){
            intakeMotor.setPower(1);
        }
        else{
            intakeMotor.setPower(0);
        }

        if(gamepad1.a == true){
            rightFlyWheelMotor.setPower(1);
            leftFlyWheelMotor.setPower(1);
        }
        else{
            leftFlyWheelMotor.setPower(0);
            rightFlyWheelMotor.setPower(0);
        }

        telemetry.addData("Left Front Motor Power", leftFrontMotor.getPowerFloat());
        telemetry.update();
    }
}
