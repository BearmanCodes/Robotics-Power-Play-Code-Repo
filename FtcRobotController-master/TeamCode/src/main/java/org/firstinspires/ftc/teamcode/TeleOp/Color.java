package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
@Disabled
@TeleOp(name="ColorTest",group="Yuppums")
public class Color extends LinearOpMode {
    ColorSensor colorSensor;
    int red;
    int green;
    int blue;


    @Override
    public void runOpMode(){
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        colorSensor.enableLed(true);
        colorLoad();

        waitForStart();

        while (opModeIsActive()){
            colorLoad();
        }
    }

    public void colorLoad(){
        red = colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();
    }

    public void telemetryColor(){
        telemetry.addData("Red: ", red);
        telemetry.addData("Green: ", green);
        telemetry.addData("Blue: ", blue);
        telemetry.update();
    }
}
