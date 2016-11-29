package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Darren Kam on 9/28/2016.
 */
@Autonomous(name = "Team 3: AutoBlue", group = "Team 3")
//@Disabled
public class Team3AutoBlue extends LinearOpMode {
	Team3Robot _robot = new Team3Robot();
    
    public void runOpMode() {
    	// Init code //
    	_robot.init(hardwareMap, telemetry);
    	_robot.setTeamColor(Team3Robot.FtcColor.BLUE);
    	
    	waitForStart();
    	
    	// Main code //
    	_robot.wait(Team3Robot.WAIT_PERIOD);
    	_robot.autoDriveDistance(Team3Robot.DISTANCE_TO_VORTEX, 1.0);
    	_robot.shootBall();
    	_robot.shootBall();
    	_robot.autoTurnInPlace(240, 0.3); //TODO - _robot.autoTurnInPlace(-135, 1.0);
    	_robot.autoDriveToBeacon();
    	_robot.autoPressBeacon();
        _robot.autoTurnInPlace(270, 1.0);
        _robot.autoDriveToBeacon();
        _robot.autoTurnInPlace(90, 1.0);
        _robot.autoPressBeacon();
    	
    	// Stop code //
    	_robot.stop();
    }
}

