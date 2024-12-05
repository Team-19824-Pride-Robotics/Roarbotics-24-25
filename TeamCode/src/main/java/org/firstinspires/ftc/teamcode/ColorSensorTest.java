package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "ColorSensorTest")
public class ColorSensorTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor color;
    int Red;
    int Green;
    int Blue;
    int redValue;
    int greenValue;
    int blueValue;
    @Override
    public void runOpMode() {
        color = hardwareMap.get(ColorSensor.class, "color");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        DigitalChannel redLED = hardwareMap.get(DigitalChannel.class, "red");
        DigitalChannel greenLED = hardwareMap.get(DigitalChannel.class, "green");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (Red > redValue){
                greenLED.setState(false);
                redLED.setState(true);
        }
            if (Blue > blueValue){
                greenLED.setState(true);
                redLED.setState(false);
            }
            if (Green > greenValue){
                greenLED.setState(true);
                redLED.setState(true);
            } else {
                redLED.setState(false);
                greenLED.setState(false);
            }


            telemetry.addData("Red: ", color.red());
            telemetry.addData("Green: ", color.green());
            telemetry.addData("Blue: ", color.blue());
            telemetry.update();
        }
    }




}

