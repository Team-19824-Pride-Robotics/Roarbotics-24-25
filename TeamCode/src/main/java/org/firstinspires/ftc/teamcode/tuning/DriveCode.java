/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class DriveCode extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private CRServo intakeServo1;
    private CRServo intakeServo2;
    private Servo armServo;
    private DcMotor elevatorMotor1;
    private DcMotor elevatorMotor2;
    private Servo slidesServo;
    private Servo specimenServo;
    private Servo bucketServo;

    public static int intake1On = 1;
    public static int intake1Off = 0;
    public static int intake1Reverse = -1;
    public static int intake2On = -1;
    public static int intake2Off = 0;
    public static int intake2Reverse = 1;
    public static int armIntakePos = 0;
    public static int armReadyPos = 0;
    public static int armOuttakePos = 0;
    public static double slidesIntake = 0;
    public static double slidesOuttake = 0;
    public static double slidesStandby = 0;
    public static int elevatorHighBucket = 0;
    public static int elevatorLowBucket = 0;
    public static int elevatorScoreSpecimen = 0;
    public static int elevatorIntakeSpecimen = 0;
    public static int elevatorDown = 0;
    public static int specimenIntake = 0;
    public static int specimenOuttake = 0;
    public static int bucketIntake = 0;
    public static int bucketOuttake = 0;
    public static int bucketMid = 0;


    @Override
    public void runOpMode() {

         int intake1Power = 0;
         int intake2Power = 0;
        double armPosition = 0;
        double slidesPosition = 0;
        int elevatorHeight = 0;
        int specimenIntakePosition = 0;
        int bucketPosition = 0;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        intakeServo1 = hardwareMap.get(CRServo.class, "intakeServo1");
        intakeServo2 = hardwareMap.get(CRServo.class, "intakeServo2");
        armServo = hardwareMap.get(Servo.class, "armServo");
        elevatorMotor1 = hardwareMap.get(DcMotor.class, "elevatorMotor1");
        elevatorMotor2 = hardwareMap.get(DcMotor.class, "elevatorMotor2");
        slidesServo = hardwareMap.get(Servo.class, "slidesServo");
        specimenServo = hardwareMap.get(Servo.class, "specimenServo");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

          leftFront.setPower(frontLeftPower);
         leftBack.setPower(backLeftPower);
         rightFront.setPower(frontRightPower);
           rightBack.setPower(backRightPower);

           if (gamepad2.a){
               intake1Power = intake1On;
               intake2Power = intake2On;
               armPosition = armIntakePos;
               slidesPosition = slidesIntake;

           }
           if(gamepad2.b){
               intake2Power = intake2Reverse;
               intake1Power = intake1Reverse;
               armPosition = armOuttakePos;
               slidesPosition = slidesOuttake;

           }
           else {
            armPosition = armReadyPos;
            slidesPosition = slidesStandby;



           }
           if (gamepad2.x){
               intake1Power = intake1On;
               intake2Power = intake2On;
           }
           if (gamepad2.y){
               intake2Power = intake2Reverse;
               intake1Power = intake1Reverse;
           }
           else{
               intake1Power = intake1Off;
               intake2Power = intake2Off;


           }
           if (gamepad2.dpad_up){
            elevatorHeight = elevatorHighBucket;
            bucketPosition = bucketMid;
           }
           if( gamepad2.dpad_down){
               elevatorHeight = elevatorDown;
               bucketPosition = bucketMid;
           }
           if (gamepad2.dpad_left){
               elevatorHeight = elevatorLowBucket;
               bucketPosition = bucketMid;
           }
           if(gamepad2.left_bumper){
               elevatorHeight = elevatorIntakeSpecimen;
               specimenIntakePosition = specimenIntake;
               bucketPosition = bucketMid;
           }
           if(gamepad2.right_bumper){
               elevatorHeight = elevatorScoreSpecimen;
               specimenIntakePosition = specimenOuttake;
               bucketPosition = bucketMid;

           }
           if(gamepad2.left_trigger > 0.5){
               bucketPosition = bucketIntake;
           }
           if(gamepad2.right_trigger > 0.5){
               bucketPosition = bucketOuttake;
           }
           if(gamepad2.start){
               bucketPosition = bucketMid;
           }

           intakeServo2.setPower(intake2Power);
           intakeServo1.setPower(intake1Power);
           armServo.setPosition(armPosition);
           slidesServo.setPosition(slidesPosition);
            elevatorMotor1.setTargetPosition(elevatorHeight);
                elevatorMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               elevatorMotor1.setPower (1);
            elevatorMotor2.setTargetPosition(elevatorHeight);
            elevatorMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorMotor2.setPower (1);
            specimenServo.setPosition(specimenIntakePosition);
            bucketServo.setPosition(bucketPosition);
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftFrontPower;
            double rightFrontPower;
            double leftBackPower;
            double rightBackPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels

            // Show the elapsed game time and wheel power.
            telemetry.update();
        }
    }
}
