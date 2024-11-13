/*
Copyright 2024

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;



/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp

public class IntakeTest extends LinearOpMode {
    private CRServo intakeServo;
    private CRServo armServo;
    private Servo bucketServo;

    public static double upPos = 0.3;
    public static double midPos = 0.6;
    public static double downPos = 0.9;
    @Override
    public void runOpMode() {

        double intakePos = 0;

        intakeServo = hardwareMap.get(CRServo.class,"intakeServo");
        armServo = hardwareMap.get(CRServo.class,"armServo");
        bucketServo = hardwareMap.get(Servo.class,"bucketServo");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.a){
                armServo.setPower(1);
                intakeServo.setPower(-1);
            }
            if (gamepad1.b){
                armServo.setPower(-1);
                intakeServo.setPower(1);
            }
            if (gamepad1.start){
                armServo.setPower(0);
                intakeServo.setPower(0);
            }
         if(gamepad1.left_bumper){
                intakePos = upPos;
            }
            if(gamepad1.right_bumper){
                intakePos = downPos;
            }
            if(gamepad1.y){
                intakePos = midPos;
            }
            telemetry.addData("Status", "Running");
            telemetry.addData("test", 2);
            telemetry.addData("arm pos =", intakePos);

            telemetry.update();
            bucketServo.setPosition(intakePos);
        }
    }
}
