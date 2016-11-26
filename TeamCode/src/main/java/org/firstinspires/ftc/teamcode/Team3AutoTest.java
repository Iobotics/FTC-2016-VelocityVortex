package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Darren Kam on 9/28/2016.
 */
@Autonomous(name = "Team 3: AutoTest", group = "Team 3")
//@Disabled
public class Team3AutoTest extends Team3Base {
    @Override
    public void robotInit() {
        this.setTeamColor(FtcColor.RED);
    }

    @Override
    public void robotMain() {
        this.wait(250);
        this.autoDriveDistance(26 + DISTANCE_OFFSET, 1.0);
        //this.autoGyroDrive(0.7, 24 + DISTANCE_OFFSET, 0.0);
        /*this.shootBall();
        this.shootBall();*/
        this.autoTurnInPlace(240, 0.3); //TODO - this.autoTurnInPlace(-135, 1.0);
        this.autoDriveToBeacon();
        this.autoPressBeacon();
        /*this.autoTurnInPlace(-90, 1.0);
        this.autoDriveToBeacon();
        this.autoPressBeacon();
        this.autoTurnInPlace(-20, 1.0);
        this.autoDriveDistance(10, 1.0);*/
    }
}

