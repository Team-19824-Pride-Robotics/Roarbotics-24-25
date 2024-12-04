package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="DriveCode")
@Config
public class DriveCode extends LinearOpMode {

    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private CRServo intakeServo1;
    private CRServo intakeServo2;
    private Servo armServo;
    private DcMotorEx liftMotor1;
    private DcMotorEx liftMotor2;
    private Servo slideServo;
    private Servo bucketServo;
    private Servo specimenArmServo;


    public static double in_speed = -1;
    public static double out_speed = 1;
    public static int lift_transfer = 0;
    public static int lift_high_bucket = -4000;
    public static int lift_low_bucket = -2000;
    public static double arm_down = 0.03;
    public static double arm_transfer = 0.74;
    public static double arm_mid = 0.15;
    public static double slides_extended = 0.82;
    public static double slides_transfer = 0.85;
    public static double slides_mid = 0.85;
    public static double bucket_transfer = 0.5;
    public static double bucket_dump = 0.02;
    public static double bucket_mid = 0.3;
    public static double dump_time = 0.5;
    public static double driveSlow = 0.5;
    public static double specimen_pickup = 0.1;
    public static double specimen_score = 0.9;

    private boolean last_A = false;
    private boolean arm_go_down = false;
    private boolean last_B = false;
    private boolean slide_go_away = false;


    @Override
    public void runOpMode() {


       double intakeSpeed = 0;
       double driveSpeed = 1;
       double armHeight = 0.74;
       double slidePosition = 0;
       double bucketPosition = 0;
       double specimenPosition = 0;

       int liftHeight = 1000;
       int target;

        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //set the motors to "coast mode" when stopped to prevent tipping
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeServo1 = hardwareMap.get(CRServo.class,"intakeServo1");
        intakeServo2 = hardwareMap.get(CRServo.class,"intakeServo2");
        armServo = hardwareMap.get(Servo.class,"armServo");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        slideServo = hardwareMap.get(Servo.class, "slideServo");
        specimenArmServo = hardwareMap.get(Servo.class, "specArmServo");

        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setDirection(DcMotorEx.Direction.REVERSE);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            /****************************************************************
             Drive code -- basic mecanum drive with a variable to allow driver to
             slow down the speed for more controlled movement near game pieces
             ****************************************************************/

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if(gamepad1.left_trigger > 0.1) {
                driveSpeed = driveSlow;
            }
            else {
                driveSpeed = 1;
            }

            leftFront.setPower(frontLeftPower * driveSpeed);
            leftBack.setPower(backLeftPower * driveSpeed);
            rightFront.setPower(frontRightPower * driveSpeed);
            rightBack.setPower(backRightPower * driveSpeed);


            /****************************************************************
             Intake code -- the intake has three components: slides, arm, and intake
             Slides have three positions: slides_extended, slides_transfer, slides_mid
             Arm has three positions: arm_down, arm_transfer, arm_mid
             Intake has two speeds: in_speed, out_speed
             ****************************************************************/

            //pressing X extends the slides and puts the arm in position
            if(gamepad2.x) {
                slidePosition = slides_extended;
                armHeight = arm_mid;
            }

            // pressing A toggles the arm position between down and mid
            if (gamepad2.a && !last_A) {
                arm_go_down = !arm_go_down;
                if (arm_go_down) {
                    armHeight = arm_down;
                } else {
                    armHeight = arm_mid;
                }
            }
            last_A = gamepad2.a;


            //bumpers control the intake spinners
            if (gamepad1.left_bumper || gamepad2.left_bumper){
                intakeSpeed = in_speed;
            }
            else if (gamepad1.right_bumper || gamepad2.right_bumper){
                intakeSpeed = out_speed;
            }
            else {
                intakeSpeed = 0;
            }

            //dPad down moves everything to transfer positions (except slides)
            if(gamepad2.dpad_down) {
                slidePosition = slides_mid;
                armHeight = arm_transfer;
                liftHeight = lift_transfer;
                bucketPosition = bucket_transfer;
            }

            // pressing B toggles the slides position between mid and transfer
            if (gamepad2.b && !last_B) {
                slide_go_away = !slide_go_away;
                if (slide_go_away) {
                    slidePosition = slides_mid;
                } else {
                    slidePosition = slides_transfer;
                }
            }
            last_B = gamepad2.b;

            //if the color sensor has detected a sample, turn the intake off
            //if it's the wrong color, outtake it automatically






            /***************************************************************
             Lift code -- the lift has three components: lift motors, specimen arm, and bucket
             Lift is controlled by a PID loop (run to position), position is set by encoders
             Bucket has three positions: transfer, mid, and dump
             Specimen arm has two positions: pickup and score
             ***************************************************************/

            if(gamepad2.right_trigger > 0.1) {
                specimenPosition = specimen_pickup;
            }
            if(gamepad2.left_trigger > 0.1) {
                specimenPosition = specimen_score;
            }
            if(gamepad2.dpad_right) {
                liftHeight = lift_low_bucket;
                bucketPosition = bucket_mid;
            }
            if(gamepad2.dpad_up) {
                liftHeight = lift_high_bucket;
                bucketPosition = bucket_mid;
            }

            if(gamepad2.right_stick_button) {
                resetRuntime();
                if (getRuntime() < dump_time) {
                    bucketPosition = bucket_dump;
                }
                else {
                    bucketPosition = bucket_mid;
                }
            }


            /***************************************************************
             Now take all the calculated variables and send the right values
             to the right motors
             ***************************************************************/

            //send the lift motors to the current value of the liftHeight variable
            target = Range.clip(liftHeight, 0, 2400);

            liftMotor1.setTargetPosition(target);
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setTargetPosition(target);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor1.setPower(1);
            liftMotor2.setPower(1);

            //send all the servos to their current positions
            intakeServo1.setPower(intakeSpeed);
            intakeServo2.setPower(-intakeSpeed);
            armServo.setPosition(armHeight);
            slideServo.setPosition(slidePosition);
            bucketServo.setPosition(bucketPosition);
            specimenArmServo.setPosition(specimenPosition);


            telemetry.addData("Status", "Running");
            telemetry.addData("lift1 height",  liftMotor1.getCurrentPosition());
            telemetry.addData("lift2 height",  liftMotor2.getCurrentPosition());
            telemetry.update();

        }
    }
}
