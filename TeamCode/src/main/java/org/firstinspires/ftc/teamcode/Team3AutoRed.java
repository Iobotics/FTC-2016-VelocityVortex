package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.SuperK9Base.TeamNumber;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name = "Team 3: AutonomousRed", group = "Team 3")
//@Disabled
public class Team3AutoRed extends OpMode {

    public enum FtcColor {
        RED,
        BLUE,
        NONE
    }

    final int SHOOTER_ROTATION = 765;
    final int TICK_OFFSET = 450; // Calibrate value
    final double LEFT_OFFSET = 0.25;
    final int ENCODER_TICKS_PER_REV = 1120;
    final int WHEEL_DIAMETER = 4;
    final double INCHES_PER_TICK = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;
    final double ROBOT_RADIUS = 8; // Inches
    final double LEFT_SERVO_HOME = 0.74;
    final double RIGHT_SERVO_HOME = 0.55;

    int leftOffset;
    int rightOffset;

    int shooterOffset;

    private FtcColor teamColor;

    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor leftBackMotor;

    DcMotor intakeMotor;
    DcMotor shooterMotor;

    Servo rightBeaconServo;
    Servo leftBeaconServo;

    DeviceInterfaceModule cdim;
    ColorSensor sensorRGB;

    @Override
    public void init() {
        teamColor = FtcColor.RED;

        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftRear");
        rightBackMotor = hardwareMap.dcMotor.get("rightRear");

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        shooterMotor = hardwareMap.dcMotor.get("shooter");

        rightBeaconServo = hardwareMap.servo.get("rightBeacon");
        leftBeaconServo = hardwareMap.servo.get("leftBeacon");

        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        sensorRGB = hardwareMap.colorSensor.get("color");

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        leftBeaconServo.scaleRange(0.132, LEFT_SERVO_HOME);
        rightBeaconServo.scaleRange(rightBeaconServo.MIN_POSITION, RIGHT_SERVO_HOME);

        leftBeaconServo.setPosition(1);
        rightBeaconServo.setPosition(1);

        this.runUsingEncoders();

        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterOffset = shooterMotor.getCurrentPosition();

        this.resetEncoders();
    }

    @Override
    public void loop() {
        this.drive(24, 1.0);
        this.shootBall();
        //this.runIntake(20);
        //this.shootBall();
        this.drive(28, 1.0);
        //this.turn(-90, 1.0);
        /*
        this.drive(43, 1.0);
        this.readBeacon(teamColor);
        this.drive(6, 1.0);
        this.resetServos();
        this.drive(-10, 1.0);
        this.turn(90, 1.0);
        this.drive(36, 1.0);*/
        requestOpModeStop();
    }

    @Override
    public void stop() {
        this.setPower(0, 0);
    }

    private void runUsingEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetEncoders() {
        leftOffset = leftFrontMotor.getCurrentPosition();
        rightOffset = rightFrontMotor.getCurrentPosition();
    }

    private void setPower(double left, double right) {
        leftFrontMotor.setPower(left - LEFT_OFFSET);
        rightFrontMotor.setPower(right - LEFT_OFFSET);
        leftBackMotor.setPower(left);
        rightBackMotor.setPower(right);
    }

    /**
     * Method to drive distance with positive power
     * @param distance
     * @param power (positive)
     */
    private void drive(double distance, double power) {
        telemetry.addData("Target pos", (distance / INCHES_PER_TICK) + TICK_OFFSET);
        telemetry.update();
        if(power < 0) throw new IllegalArgumentException("power = " + power);
        if(distance < 0) {
            power = -power;
            distance = -distance;
        }
        this.resetEncoders();
        while(leftFrontMotor.getCurrentPosition() - leftOffset < (distance / INCHES_PER_TICK) + TICK_OFFSET) {
            this.setPower(power, power);
            telemetry.addData("Current pos", leftFrontMotor.getCurrentPosition());
            telemetry.update();
        }
        this.setPower(0, 0);
        this.resetEncoders();
    }

    /**
     * Method to turn degrees with positive power
     * @param degrees
     * @param power
     */
    // FIXME - Create a turn method
    private void turn(int degrees, double power) {
        if(power < 0) throw new IllegalArgumentException("power = " + power);
        if(degrees < 0) {
            power = -power;
            degrees = -degrees;
        }

        double distance = (degrees * (Math.PI / 180) * ROBOT_RADIUS) / (WHEEL_DIAMETER * Math.PI) * ENCODER_TICKS_PER_REV;

        while(leftFrontMotor.getCurrentPosition() - leftOffset < distance) {
            this.setPower(power, -power);
        }
        this.setPower(0, 0);
        this.resetEncoders();
    }

    private void shootBall() {
        while((shooterMotor.getCurrentPosition() - shooterOffset) < SHOOTER_ROTATION) {
            shooterMotor.setPower(1);
        }
        shooterMotor.setPower(0);
        shooterOffset = shooterMotor.getCurrentPosition();
    }

    private void runIntake(int seconds) {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        time.reset();
        while(time.seconds() < seconds) {
            intakeMotor.setPower(1);
        }
        intakeMotor.setPower(0);
    }

    // FIXME - Find color sensor threshold
    private void readBeacon(FtcColor teamColor) {
        if(teamColor == FtcColor.RED && sensorRGB.red() >= 3000 && sensorRGB.blue() <= 3000) {
            leftBeaconServo.setPosition(Servo.MIN_POSITION);
        } else {
            rightBeaconServo.setPosition(Servo.MIN_POSITION);
        }
    }

    private void resetServos() {
        leftBeaconServo.setPosition(1);
        rightBeaconServo.setPosition(1);
    }
}