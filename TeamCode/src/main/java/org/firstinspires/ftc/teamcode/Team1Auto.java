package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDevice;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbLegacyModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

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

    ModernRoboticsI2cGyro gyro;


    int catapultOffset;
    int rightMotorOffset;
    int leftMotorOffset;

    int targetRotations;
    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");

        catapultMotor = hardwareMap.dcMotor.get("catapult");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.i2cDevice.get("gyro");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        catapultMotor.setDirection(DcMotor.Direction.REVERSE);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        catapultMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        rightMotorOffset = backRightMotor.getCurrentPosition();
        catapultOffset = catapultMotor.getCurrentPosition();
        leftMotorOffset = backLeftMotor.getCurrentPosition();

    }

    @Override
    public void loop() {

        this.moveRobot(12, 0.3);

        this.activateCatapult();

        this.rotate(90, 0.3 ,'l');

        this.moveRobot(12,0.3);

        requestOpModeStop();
    }

    @Override
    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private void moveRobot(int distance, double power){
        while(backRightMotor.getCurrentPosition() - rightMotorOffset < targetPosition(distance)) {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(power);
        }

        rightMotorOffset = backRightMotor.getCurrentPosition();
        catapultOffset = catapultMotor.getCurrentPosition();
        leftMotorOffset = backLeftMotor.getCurrentPosition();

        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);

    }

    private void activateCatapult(){
        while (catapultMotor.getCurrentPosition() -catapultOffset < ENCODER_TICKS_PER_REV * 3) {
            catapultMotor.setPower(1);
        }

        catapultOffset = catapultMotor.getCurrentPosition();
        rightMotorOffset = backRightMotor.getCurrentPosition();
        leftMotorOffset = backLeftMotor.getCurrentPosition();

        catapultMotor.setPower(0);
    }

    /**
     * Method to convert inches to ticks
     * @param distance
     * @return distanceInTicks
     */
    private int targetPosition(double distance){
        double distanceInTicks = distance / INCHES_PER_TICK;
        return (int) distanceInTicks;
    }

    private void rotate(int degrees, double power, char direction){
        targetRotations = targetPosition(24 * Math.PI / (360/degrees));
        double degreesInRotationFraction = degrees/360;

        gyro.calibrate();
        while(gyro.isCalibrating() == true){}

        if(direction == 'l') {
            while (gyro.getRotationFraction() < degreesInRotationFraction) {
                frontLeftMotor.setPower(power);
                frontRightMotor.setPower(-power);
                backLeftMotor.setPower(power);
                backRightMotor.setPower(-power);
            }
        }
        else if (direction == 'r'){
            while (gyro.getRotationFraction() < degreesInRotationFraction) {
                frontLeftMotor.setPower(-power);
                frontRightMotor.setPower(power);
                backLeftMotor.setPower(-power);
                backRightMotor.setPower(power);
            }
        }
        else{}
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);

        catapultOffset = catapultMotor.getCurrentPosition();
        rightMotorOffset = backRightMotor.getCurrentPosition();
        leftMotorOffset = backLeftMotor.getCurrentPosition();
    }
}
