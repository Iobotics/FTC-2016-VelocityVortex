package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Darren Kam on 9/28/2016.
 */
@Autonomous(name = "Team 3: AutoBlue", group = "Team 3")
//@Disabled
public class Team3AutoBlue extends Team3Robot {	
	@Override
	public void robotInit() {
		this.setTeamColor(FtcColor.BLUE);
	}
	
	@Override
	public void robotMain() {
		this.wait(Team3Robot.WAIT_PERIOD);

		this.autoDriveDistance(Team3Robot.DISTANCE_TO_VORTEX, 1.0);

        while(opModeIsActive() && this.getShooterPosition() < SHOOTER_ROTATION / 2) {
            _shooterMotor.setPower(1);
        }
        _shooterMotor.setPower(0);

        this.wait(250);

		this.shootBall();
		this.shootBall();

		// Ramp code //

        /*this.autoTurnInPlace(-30, 0.3);
        this.autoDriveDistance(42, 1.0);*/

		//          //

		/*this.autoTurnInPlace(-120, 0.3); //TODO - _robot.autoTurnInPlace(-135, 1.0);

		this.autoDriveToBeacon();
		this.autoPressBeacon();

		/*this.autoTurnInPlace(-90, 1.0);

		this.autoDriveToBeacon();
		this.autoPressBeacon();

		this.autoTurnInPlace(-37, 1.0);

		this.autoDriveDistance(20, 1.0);*/
	}
	
	@Override
	public void robotStop() { }
}

