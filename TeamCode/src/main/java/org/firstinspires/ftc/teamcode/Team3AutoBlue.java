package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Darren Kam on 9/28/2016.
 */
@Autonomous(name = "Team 3: AutoBlue", group = "Team 3")
//@Disabled
public class Team3AutoBlue extends Team3Base {
    @Override
    public void robotInit() {
        _teamColor = FtcColor.BLUE;
    }

    @Override
    public void robotMain() {
        this.autoDriveDistance(28, 1.0, 1.0);
        this.shootBall();
        this.shootBall();
        this.autoTurnInPlace(-135, 1.0);
        this.autoDriveToBeacon();
        this.autoPressBeacon();
        this.autoTurnInPlace(-90, 1.0);
        this.autoDriveToBeacon();
        this.autoPressBeacon();
        this.autoTurnInPlace(-20, 1.0);
        this.autoDriveDistance(10, 1.0);
    }
}

