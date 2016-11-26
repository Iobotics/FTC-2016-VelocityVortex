package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.ams.AMSColorSensorImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Created by Teacher on 10/5/2016.
 */

@TeleOp(name = "Color Sensor Test", group = "Color Sensor")
//@Disabled
public class ColorSensorTest extends OpMode {

    ColorSensor sensorRGB;
    DeviceInterfaceModule cdim;

    static final int LED_CHANNEL = 5;

    @Override
    public void init() {
        //sensorRGB = (AMSColorSensorImpl) AMSColorSensorImpl.create(AMSColorSensor.Parameters.createForAdaFruit(), hardwareMap.i2cDevice.get("color"));
        sensorRGB = hardwareMap.colorSensor.get("color");

        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelState(LED_CHANNEL, false);
    }

    @Override
    public void loop() {
        telemetry.addData("red", sensorRGB.red());
        telemetry.addData("green", sensorRGB.green());
        telemetry.addData("blue", sensorRGB.blue());
        telemetry.addData("info", sensorRGB.getConnectionInfo());
        telemetry.addData("address", sensorRGB.getI2cAddress());
        telemetry.addData("name", sensorRGB.getDeviceName());
        telemetry.update();
    }
}
