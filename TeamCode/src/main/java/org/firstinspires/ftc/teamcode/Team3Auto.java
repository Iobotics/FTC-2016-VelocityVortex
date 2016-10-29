package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.SuperK9Base.TeamNumber;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name = "Team 3: Autonomous", group = "Team 3")
//@Disabled
public class Team3Auto extends OpMode {
	
	private enum FtcColor {
        RED,
        BLUE,
        NONE
    }

	final int TARGET_POS = 1120; // TODO - Calibrate value
    final double LEFT_SERVO_HOME = 0.45;
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
    
    public Team3Auto(FtcColor teamColor) {
    	this.teamColor = teamColor;
    }
    
    @Override
    public void init() {
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
        
        this.runUsingEncoders();
        
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterOffset = shooterMotor.getCurrentPosition();

        leftBeaconServo.scaleRange(Servo.MIN_POSITION, LEFT_SERVO_HOME);
        rightBeaconServo.scaleRange(Servo.MIN_POSITION, RIGHT_SERVO_HOME);
        
        leftBeaconServo.setPosition(1);
        rightBeaconServo.setPosition(1);

        gamepad1.setJoystickDeadzone((float) .1);
    }

    // TODO - Finish filling in autonomous
    @Override
    public void loop() {
        this.driveForward(48, 1.0);

        
        this.driveForward(43, 1.0);
        this.pressBeacon(teamColor);
        this.driveForward(6, 1.0);
        requestOpModeStop();
    }
    
    @Override
    public void stop() {
    	this.setPower(0);
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
    
    private void setPower(double power) {
    	leftFrontMotor.setPower(power);
    	leftBackMotor.setPower(power);
    	rightFrontMotor.setPower(power);
    	rightBackMotor.setPower(power);
    }
    
    private void driveForward(double distance, double power) {
    	this.resetEncoders();
    	while(leftFrontMotor.getCurrentPosition() - leftOffset < distance) {
    		this.setPower(power);
    	}
    	this.setPower(0);
    }
    
    private void turn(int degrees, double power) {
    	
    }
    
    private void pressBeacon(FtcColor teamColor) {
    	if(teamColor == FtcColor.RED && sensorRGB.red() >= 3000) {
    		leftBeaconServo.setPosition(Servo.MIN_POSITION);
    	} else {
    		rightBeaconServo.setPosition(Servo.MIN_POSITION);
    	}
    }
}

