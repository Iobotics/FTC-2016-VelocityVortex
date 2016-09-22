/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

import java.lang.Math;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class SuperK9TeleOp extends SuperK9Base {


    @Override
    public void k9Init() {
        //this.calibrateGyro();
    }

    /*
     * Autonomous program
     *
     * Move forward 2 feet
     */
    @Override
    public void k9Start() {
        this.setManServoPosition(ManServoPosition.HOME);
        //this.runWithoutEncoders();
        this.runWithEncoders();
        this.resetLightSensors();
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void k9Loop() {
        boolean disableDrive = false;

        if(gamepad1.dpad_left) {
            this.setLeftTriggerDeployed(true);
            this.setRightTriggerDeployed(false);
        } else if(gamepad1.dpad_right) {
            this.setRightTriggerDeployed(true);
            this.setLeftTriggerDeployed(false);
        } else if(gamepad1.dpad_up){
            this.setLeftTriggerDeployed(false);
            this.setRightTriggerDeployed(false);
        }

        this.setPlowPower(gamepad1.left_bumper? 1 : gamepad1.left_trigger > 0.5 ? -1 : 0);
        this.setDozerPower(gamepad1.right_bumper ? 1 : gamepad1.right_trigger > 0.5 ? -1 : 0);

        if(gamepad1.y) {
            this.setManServoPosition(ManServoPosition.DEPLOY);
        } else if(gamepad1.b || gamepad1.right_bumper){
            this.setManServoPosition(ManServoPosition.HOME);
        } else if(gamepad1.a) {
            this.setManServoPosition(ManServoPosition.PARK);
        }

        if(gamepad1.x) {
            this.setWinchPower(1.0);
        } else {
            this.setWinchPower(0.0);
        }
        if(gamepad1.right_stick_button && gamepad1.left_stick_button) {
            //this.startLaunchMotor();
            if(this.getTeamNumber() != TeamNumber.TEAM_8898) {
                this.setRightTriggerDeployed(true);
            }
            this.setLaunchReleasePower(1.0);
            this.setDozerPower(0.25);
            // disable drive //
            disableDrive = true;
        } else if(gamepad1.start) {
            this.setLaunchReleasePower(-0.5);
        } else {
            this.setLaunchReleasePower(0);
        }

        if(disableDrive) {
            this.setPower(0, 0);
        } else if(this.getTeamNumber() == TeamNumber.TEAM_8898) {
            double power = -gamepad1.right_stick_y;
            double turn  = -gamepad1.left_stick_x;
            this.setPowerArcade(power, turn, true);
        } else {
            // tank drive //
            float left = -gamepad1.left_stick_y;
            float right = -gamepad1.right_stick_y;
            this.setPowerScaled(left, right);
        }

        /*if(gamepad1.start & !this.isGyroCalibrating()) {
            this.resetGyroHeading();
        }*/

        telemetry.addData("lightOuter", this.getLightOuter());
        telemetry.addData("lightInner", this.getLightInner());
        //telemetry.addData("ods", String.format("%f, %d", _ods.getLightDetected(), _ods.getLightDetectedRaw()));
        //telemetry.addData("analog", _analog.getValue());
    }
}