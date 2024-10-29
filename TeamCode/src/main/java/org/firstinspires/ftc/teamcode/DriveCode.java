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

    // Declare OpMode members.
   // private DcMotor arm;
   // private DcMotor intake;
private DcMotor leftBack;
   private DcMotor rightBack;
   private DcMotor leftFront;
   private DcMotor rightFront;
    private CRServo intakeServo;
    private Servo armServo;
    private DcMotor elevatorMotor;
    //private Servo boxServo;
    private Servo slideServo;
    private Servo bucketServo;

    //variables for arm and bucket height
    //JakeTake Values //ed
//    public static int scoringHeight = 235;
//    public static int drivingHeight = 1411;
//    public static int startingHeight = 0;
//    public static int floorHeight = 1604;
    public static double inspeed = -1;
    public static double outspeed = 0.3;
    public static int startingHeight = 0;
    public static int scoringHeight = 100;
    public static int maxHeight = 0;
    public static int pickupHeight = -100;
    public static double inHeight = 0.03;
    public static double outHeight = 0.65;
    public static double midHeight = 0.25;
    public static double slidesOut = 0.3;
    public static double slidesIn = 0.55;
    public static double bucketIntake = 0;
    public static double bucketOut = 0;



    @Override
    public void runOpMode() {

       double speed = 0;
       double armHeight = 0.65;
       double slidePosition = 0.55;
       double bucketPosition = 0;



       // arm = hardwareMap.get(DcMotor.class, "arm");
      //  intake = hardwareMap.get(DcMotor.class, "Intake");
        //control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        //expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
       rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        intakeServo = hardwareMap.get(CRServo.class,"intakeServo");
        armServo = hardwareMap.get(Servo.class,"armServo");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        slideServo = hardwareMap.get(Servo.class, "slideServo");
       // imu = hardwareMap.get(IMU.class, "imu");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Wait for the game to start (driver presses START)
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

//             Denominator is the largest motor power (absolute value) or 1
//             This ensures all the powers maintain the same ratio,
//             but only if at least one is out of the range [-1, 1]
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
                speed = inspeed;

            }
            if (gamepad1.b || gamepad2.b){

                speed = outspeed;
            }
            if (gamepad2.start) {
               speed = 0;
            }
            // Outake position
            if (gamepad2.y){
                armHeight = (inHeight);
            }
            //Intake Position
            if (gamepad2.x){
                armHeight = (outHeight);
            }
            // Middle position
            if (gamepad2.right_stick_button){
                armHeight = (midHeight);
            }

            if (gamepad2.dpad_up){
                elevatorMotor.setTargetPosition(startingHeight);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorMotor.setPower (0.5);
            }
            if (gamepad2.dpad_down){
                elevatorMotor.setTargetPosition(scoringHeight);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorMotor.setPower (0.5);
            }
            if (gamepad2.dpad_left){
                elevatorMotor.setTargetPosition(maxHeight);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorMotor.setPower (0.5);
            }
            if (gamepad2.dpad_right){
                elevatorMotor.setTargetPosition(pickupHeight);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorMotor.setPower (0.5);
            }
            if (gamepad2.left_bumper){
                slidePosition = slidesIn;
            }
            if (gamepad2.right_bumper){
                slidePosition = slidesOut;
            }
            if (gamepad2.left_trigger > 0.5){
                bucketPosition = bucketIntake;
            }
            if (gamepad2.right_trigger > 0.5){
                bucketPosition = bucketOut;
            }



            intakeServo.setPower(speed);
          armServo.setPosition(armHeight);
          slideServo.setPosition(slidePosition);
          bucketServo.setPosition(bucketPosition);




//            if (gamepad1.y) {
//                arm.setTargetPosition(floorHeight);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setPower (0.5);
//            }
//
//            if (gamepad1.left_bumper) {
//                arm.setTargetPosition(drivingHeight);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setPower(0.5);
//            }
//
//            else if (gamepad1.right_bumper){
//                arm.setTargetPosition(startingHeight);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setPower(0.5);
//            }


            telemetry.addData("Status", "Running");
           // telemetry.addData ("ArmHeight:", arm.getCurrentPosition());
            telemetry.addData("speed",  speed);
            telemetry.update();

        }
    }
}
