package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Darren Kam on 9/28/2016.
 */
@Autonomous(name = "Team 3: AutoTest", group = "Team 3")
//@Disabled
public class Team3AutoTest extends Team3Robot {    
    @Override
	public void robotInit() {
		this.setTeamColor(FtcColor.RED);
	}
	
	@Override
	public void robotMain() {
    	this.wait(Team3Robot.WAIT_PERIOD);
    	
    	this.autoDriveDistance(Team3Robot.DISTANCE_TO_VORTEX, 1.0);
    	
    	this.shootBall();
    	this.shootBall();
    	
    	this.autoTurnInPlace(120, 0.3); //TODO - _robot.autoTurnInPlace(-135, 1.0);
    	
    	this.autoDriveToBeacon();
    	this.autoPressBeacon();
    	
        /*this.autoTurnInPlace(90, 0.3);

        this.autoDriveToBeacon();
        this.autoPressBeacon();
        
        this.autoTurnInPlace(37, 1.0);
        
        this.autoDriveDistance(20, 1.0);*/
	}
	
	@Override
	public void robotStop() { }
}

