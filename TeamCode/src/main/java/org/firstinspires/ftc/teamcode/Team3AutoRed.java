package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Darren Kam on 9/28/2016.
 */
@Autonomous(name = "Team 3: AutoRed", group = "Team 3")
//@Disabled
public class Team3AutoRed extends Team3Base {
    @Override
    public void robotInit() {
        teamColor = FtcColor.RED;
    }

    @Override
    public void robotLoop() {
        this.autoDriveDistance(28, 1.0, 1.0);
        this.shootBall();
        this.runIntake(5); // Rotates intake 5 times
        this.shootBall();
        this.autoTurnInPlace(135, 1.0);
        this.autoDriveToBeacon();
        this.autoPressBeacon();
        this.autoTurnInPlace(90, 1.0);
        this.autoDriveToBeacon();
        this.autoPressBeacon();
        this.autoTurnInPlace(20, 1.0);
        this.autoDriveDistance(10, 1.0);
        requestOpModeStop();
    }
}

