package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Teacher on 9/28/2016.
 */

@Autonomous(name = "Team 3: AutoRed", group = "Team 3")
//@Disabled
public class Team3AutoRed extends Team3Auto {
    @Override
    public void init() {
        super.init();
        teamColor = FtcColor.RED;
    }

    @Override
    public void loop() {
        this.autoDriveDistance(28, 1.0, 1.0);
        this.shootBall();
        this.runIntake(5);
        this.shootBall();
        this.autoDriveDistance(24, 0.6, 1.0);
        /*
        this.autoTurnInPlace(-90, 1.0);
        this.autoDriveDistancePID(43, 1.0);
        this.readBeacon(teamColor);
        this.autoDriveDistancePID(6, 1.0);
        this.resetServos();
        this.autoDriveDistancePID(-10, 1.0);
        this.turn(90, 1.0);
        this.autoDriveDistancePID(36, 1.0);
        */
        requestOpModeStop();
    }
}

