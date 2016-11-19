package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name = "Team 3: AutoBlue", group = "Team 3")
//@Disabled
public class Team3AutoBlue extends Team3Auto {
    @Override
    public void init() {
        super.init();
        teamColor = FtcColor.BLUE;
    }

    @Override
    public void loop() {
        this.autoDriveDistance(28, 1.0, 1.0);
        this.shootBall();
        this.runIntake();
        this.shootBall();
        this.autoTurnInPlace(-135, 1.0);
        this.autoDriveToBeacon();
        this.autoPressBeacon();
        this.autoTurnInPlace(-90, 1.0);
        this.autoDriveToBeacon();
        this.autoPressBeacon();
        this.autoTurnInPlace(-20, 1.0);
        this.autoDriveDistance(10, 1.0);
        requestOpModeStop();
    }
}

