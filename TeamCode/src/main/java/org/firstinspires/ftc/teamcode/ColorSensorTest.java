package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Created by Teacher on 10/5/2016.
 */

@TeleOp(name = "Color Sensor Test", group = "Color Sensor")
//@Disabled
public class ColorSensorTest extends OpMode {

    ModernRoboticsI2cColorSensor colorSensorL;
    DeviceInterfaceModule cdim;
    String color;
    static final int LED_CHANNEL = 5;
    int redC;
    int greenC;
    int blueC;

    @Override
    public void init() {
        colorSensorL = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("color");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelState(LED_CHANNEL, false);
        colorSensorL.setI2cAddress();

    }

    @Override
    public void loop() {
        telemetry.addData("red", colorSensorL.red()); //21504
        telemetry.addData("green", colorSensorL.green());
        telemetry.addData("blue", colorSensorL.blue());
        if(colorSensorL.red() > 1400 && colorSensorL.blue() <1500 && colorSensorL.green()< 1000){
            color= "red";
        }
        else if(colorSensorL.red() < 1000 && colorSensorL.blue() >2000 && colorSensorL.green()< 1000){
            color = "blue";
        }
        else{
            color = "not red or blue";
        }
        telemetry.addData("color", color);
        telemetry.addData("info", colorSensorL.getConnectionInfo());
        telemetry.addData("info", colorSensorL.getI2cAddress());
        telemetry.addData("Name", colorSensorL.getDeviceName());
        telemetry.update();
    }
}
