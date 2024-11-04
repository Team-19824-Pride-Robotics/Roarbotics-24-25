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

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="DriveCode")
//@Disabled
@Config
public class DriveCode extends LinearOpMode {

    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private CRServo intakeServo;
    private Servo armServo;
    private DcMotor elevatorMotor;
    private Servo slideServo;
    private Servo bucketServo;

    public static double inSpeed = -1;
    public static double outSpeed = 1;
    public static int startingHeight = 0;
    public static int highBucketHeight = -4000;
    public static int lowBucketHeight = -2000;
    public static double inHeight = 0.03;
    public static double outHeight = 0.63;
    public static double midHeight = 0.25;
    public static double slidesOut = 0.82;
    public static double slidesIn = 0.85;
    public static double bucketIntake = 0.5;
    public static double bucketOut = 0.02;
    public static double bucketMid = 0.3;

    @Override
    public void runOpMode() {

       double speed = 0;
       double armHeight = 0.63;
       double slidePosition = 1;
       double bucketPosition = 0.3;

        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        intakeServo = hardwareMap.get(CRServo.class,"intakeServo");
        armServo = hardwareMap.get(Servo.class,"armServo");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        slideServo = hardwareMap.get(Servo.class, "slideServo");

        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (isStopRequested()) return;

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

            if (gamepad1.a || gamepad2.a){
                speed = inSpeed;
            }
            if (gamepad1.b || gamepad2.b){
                speed = outSpeed;
            }
            if (gamepad2.start || gamepad1.options) {
               speed = 0;
            }
            // Outake position
            if (gamepad2.y){
                armHeight = inHeight;
            }
            //Intake Position
            if (gamepad2.x){
                armHeight = outHeight;
            }
            // Middle position
            if (gamepad2.right_stick_button){
                armHeight = midHeight;
            }

            if (gamepad2.dpad_down){
                elevatorMotor.setTargetPosition(startingHeight);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorMotor.setPower (1);
            }
            if (gamepad2.dpad_up){
                elevatorMotor.setTargetPosition(highBucketHeight);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorMotor.setPower (1);
            }

            if (gamepad2.dpad_right){
                elevatorMotor.setTargetPosition(lowBucketHeight);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorMotor.setPower (1);
            }
            if (gamepad2.left_bumper){
                slidePosition = slidesIn;
                armHeight = outHeight;
                bucketPosition = bucketIntake;
            }
            if (gamepad2.right_bumper){
                slidePosition = slidesOut;
                armHeight = inHeight;
                bucketPosition = bucketMid;
            }
            if (gamepad2.left_trigger > 0.5){
                bucketPosition = bucketIntake;
            }
            if (gamepad2.right_trigger > 0.5){
                bucketPosition = bucketOut;
            }
            if (gamepad2.left_stick_button){
                bucketPosition = bucketMid;
            }


          intakeServo.setPower(speed);
          armServo.setPosition(armHeight);
          slideServo.setPosition(slidePosition);
          bucketServo.setPosition(bucketPosition);


            telemetry.addData("Status", "Running");
            telemetry.addData("lift height",  elevatorMotor.getCurrentPosition());
            telemetry.update();

        }
    }
}
