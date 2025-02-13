package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

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
    private DcMotorEx slidesMotor;
    private Servo bucketServo;
    private Servo specimenArmServo;
    private Servo blockServo;
    private Servo clawServo;


    public static double in_speed = 1;
    public static double fast_out_speed = -1;
    public static double slow_out_speed = -0.3;
    public static int lift_transfer = 0;
    public static int lift_high_bucket = -3200;
    public static int lift_low_bucket = -1000;
    public static int lift_spec_pickup = -400;
    public static int lift_spec_score = -900;
    public static int lift_high_hang = -3500;
    public static int lift_low_hang = -2500;
    public static double arm_down = 0.1;
    public static double arm_transfer = 0.9;
    public static double arm_mid = 0.35;
    public static int slides_extended = -100;
    public static int slides_transfer = -30;
    public static int slides_mid = -80;
    public static double bucket_transfer = 0;
    public static double bucket_dump = 0.85;
    public static double bucket_mid = 0.3;


    public static double dump_time = 0.5;
    public static double driveSlow = 0.5;
    public static double specimen_pickup = 0;
    public static double specimen_score = 0.66;
    public static double block_open = 0;
    public static double claw_open = 0.5;
    public static double claw_closed = 0.85;
    public static double block_closed = 0.4;

    private boolean last_A = false;
    private boolean arm_go_down = false;
    private boolean last_B = false;
    private boolean slide_go_away = false;

    private DigitalChannel greenLED;
    private DigitalChannel redLED;
    ColorSensor color_sense;


    @Override
    public void runOpMode() {

       double intakeSpeed = 0;
       double driveSpeed = 1;
       double armHeight = 0.1;
       int slidePosition = 0;
       double bucketPosition = 0;
       double specimenPosition = 0.4;
       double out_speed = -1;
       double claw_position = 0.5;


        int red;
        int blue;
        int green;
        double sample;

       int liftHeight = 0;
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
        specimenArmServo = hardwareMap.get(Servo.class, "specArmServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        slidesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        color_sense = hardwareMap.get(ColorSensor.class, "color_sense");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED = hardwareMap.get(DigitalChannel.class, "red");
        redLED.setMode(DigitalChannel.Mode.OUTPUT);

        blockServo.setPosition(block_open);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //go get the values from the color sensor
            red = color_sense.red();
            blue = color_sense.blue();
            green = color_sense.green();
            sample = ((OpticalDistanceSensor)color_sense).getLightDetected();

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
            if(gamepad2.y){
                claw_position = claw_open;
            }

            if (gamepad2.dpad_left){
                claw_position = claw_closed;
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
            if (gamepad1.right_bumper || gamepad2.right_bumper){
                intakeSpeed = in_speed;
            }
            else if (gamepad1.left_bumper || gamepad2.left_bumper){
                intakeSpeed = out_speed;
            }
/*
            //if the color sensor detects the wrong color for our alliance, spit out the sample

            else if (red > 1000 && green < 3000) {
                intakeSpeed = out_speed;
            }

 */

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

            //sets the outspeed to slower
            if (gamepad1.right_trigger > 0.5){
                out_speed = slow_out_speed;
            }
            else {
                out_speed = fast_out_speed;
            }



            //if the color sensor has detected a BLUE or YELLOW sample, turn on LED indicator light
            // so the drivers know they have it

            greenLED.setState(true);
           // redLED.set
         //   blue > 100 || green > 50



            /***************************************************************
             Lift code -- the lift has three components: lift motors, specimen arm, and bucket
             Lift is controlled by a PID loop (run to position), position is set by encoders
             Bucket has three positions: transfer, mid, and dump
             Specimen arm has two positions: pickup and score
             ***************************************************************/


            if(gamepad2.dpad_right) {
                liftHeight = lift_low_bucket;
                bucketPosition = bucket_mid;
            }
            if(gamepad2.dpad_up) {
                liftHeight = lift_high_bucket;
                bucketPosition = bucket_mid;
                slidePosition = slides_mid;
            }
            if(gamepad1.a) {
                liftHeight = lift_high_hang;
            }
            if(gamepad1.b) {
                liftHeight = lift_low_hang;
            }

            if(gamepad2.right_trigger > 0.1) {
                    bucketPosition = bucket_dump;
                }

            if(gamepad2.left_trigger > 0.1) {
                bucketPosition = bucket_mid;
            }

            if(gamepad2.left_stick_button) {
                specimenPosition = specimen_score;
                liftHeight = lift_spec_score;
            }
            else {
                specimenPosition = specimen_pickup;
            }
            if(gamepad2.right_stick_button) {
                liftHeight = lift_spec_pickup;
            }



            /***************************************************************
             Now take all the calculated variables and send the right values
             to the right motors
             ***************************************************************/

            //send the lift motors to the current value of the liftHeight variable
           // target = Range.clip(liftHeight, 0, 2400);

            liftMotor1.setTargetPosition(liftHeight);
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setTargetPosition(liftHeight);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor1.setPower(1);
            liftMotor2.setPower(1);

            slidesMotor.setTargetPosition(slidePosition);
            slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesMotor.setPower(1);

            //send all the servos to their current positions
            intakeServo1.setPower(intakeSpeed);
            intakeServo2.setPower(-intakeSpeed);
            armServo.setPosition(armHeight);
            bucketServo.setPosition(bucketPosition);
            specimenArmServo.setPosition(specimenPosition);
            clawServo.setPosition(claw_position);


            telemetry.addData("Status", "Running");
            telemetry.addData("RED", red);
            telemetry.addData("BLUE", blue);
            telemetry.addData("GREEN", green);
            telemetry.addData("COLOR", sample);
            telemetry.addData("slideposition", slidesMotor.getCurrentPosition());
//            telemetry.addData("lift1 height",  liftMotor1.getCurrentPosition());
//            telemetry.addData("lift2 height",  liftMotor2.getCurrentPosition());
            telemetry.update();

        }
    }
}
