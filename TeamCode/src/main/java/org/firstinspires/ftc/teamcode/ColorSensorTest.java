package org.firstinspires.ftc.teamcode;

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

    AdafruitI2cColorSensor colorSensorL;
    DeviceInterfaceModule cdim;

    static final int LED_CHANNEL = 5;

    @Override
    public void init() {
        colorSensorL = (AdafruitI2cColorSensor) hardwareMap.colorSensor.get("colorSensor");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelState(LED_CHANNEL, false);
    }

    @Override
    public void loop() {
        telemetry.addData("red", colorSensorL.red());
        telemetry.addData("green", colorSensorL.green());
        telemetry.addData("blue", colorSensorL.blue());
        //telemetry.addData("color", ) // TODO - Differentiate between red and blue
        telemetry.addData("info", colorSensorL.getConnectionInfo());
        telemetry.update();
    }
}
