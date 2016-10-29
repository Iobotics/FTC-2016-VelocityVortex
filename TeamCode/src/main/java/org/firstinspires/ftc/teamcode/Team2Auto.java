package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name = "Team 2: Autonomous", group = "Team 2")
//@Disabled
public class Team2Auto extends OpMode {
	
	final int    ENCODER_TICKS_PER_REV = 1120; // Neverest 40
	final int 	 CATAPULT_TICKS 	   = 3 * ENCODER_TICKS_PER_REV; // Three rotations
	final int    CHASSIS_DIAMETER	   = 18; // CM
    final int    WHEEL_DIAMETER        = 10; // CM
    final double CM_PER_TICK		   = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV; // CM / REV
	
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    
    DcMotor intakeMotor;
    DcMotor catapultMotor;

    int leftOffset;
    int rightOffset;
    
    int catapultOffset;
    
    ElapsedTime time;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");
        
        catapultMotor = hardwareMap.dcMotor.get("catapult");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        catapultMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        time.reset();

        // Runs intake and shooter on start for 5 seconds
        while (time.seconds() < 5){
            intakeMotor.setPower(1.0);
        }
        intakeMotor.setPower(0.0);
    }

    @Override
    public void loop() {
        moveForward(127, 1.0); // Move forward 127 cm at 1.0 power
        activateCatapult();
        moveForward(23, 1.0); // Move forward 23 cm at 1.0 power
        
        requestOpModeStop();
    }
    
    @Override
    public void stop() {
    	frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    
    /**
     * A method to calculate the number of ticks to move the robot a number of inches
     * @param distance
     * @return distanceInTicks
     */
    private int distance(double distance){
        double distanceInTicks = distance / CM_PER_TICK;
        return (int) distanceInTicks;
    }

    /**
     * A method to calculate amount of ticks to rotate a specified number of degrees
     * @param degrees
     * @return distanceInTicks
     */
    private int rotation(int degrees){
        double chassisCircumference = CHASSIS_DIAMETER * Math.PI;
        double centimetersPerDegree = chassisCircumference / 360;
        double distanceInCm = centimetersPerDegree * degrees;
        double distanceInTicks = distanceInCm / CM_PER_TICK;
        return (int) distanceInTicks;
    }

    /**
     * Method that moves a target distance at a target power
     * @param targetDistance
     * @param targetPower
     */
    private void moveForward(int targetDistance, double targetPower){
        int distanceToTravel = distance(targetDistance);

        while((frontLeftMotor.getCurrentPosition() - leftOffset) < distanceToTravel) {
            frontLeftMotor.setPower(targetPower);
            frontRightMotor.setPower(targetPower);
            backLeftMotor.setPower(targetPower);
            backRightMotor.setPower(targetPower);
        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        
        leftOffset = frontLeftMotor.getCurrentPosition();
    }

    /**
     * Method to fire the catapult
     */
    private void activateCatapult() {
        while((catapultMotor.getCurrentPosition() - catapultOffset) < CATAPULT_TICKS) {
            catapultMotor.setPower(1);
        }
        catapultMotor.setPower(0);
        catapultOffset = catapultMotor.getCurrentPosition();
    }
}
