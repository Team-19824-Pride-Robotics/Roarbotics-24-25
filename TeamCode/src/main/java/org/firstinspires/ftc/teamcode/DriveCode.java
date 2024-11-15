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
    private CRServo intakeServo1;
    private CRServo intakeServo2;
    private Servo armServo;
    private DcMotor elevatorMotor;
    private Servo slideServo;
    private Servo bucketServo;

    public static double intakeSpeed = -1;
    public static double intakeOutSpeed = 1;
    public static int startingHeight = 0;
    public static int highBucketHeight = -4000;
    public static int lowBucketHeight = -2000;
    public static double armIntakeHeight = 0.03;
    public static double armOuttakeHeight = 0.63;
    public static double armMidHeight = 0.25;
    public static double slidesExtended = 0.82;
    public static double slidesIn = 0.85;
    public static double bucketIntake = 0.5;
    public static double bucketDump = 0.02;
    public static double bucketMid = 0.3;

    @Override
    public void runOpMode() {

       double speed = 0;
       double armHeight = 0.63;
       double slidePosition = 1;
       double bucketPosition = 0.3;

//        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
//        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
//        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
//        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        intakeServo1 = hardwareMap.get(CRServo.class,"intakeServo1");
        intakeServo2 = hardwareMap.get(CRServo.class,"intakeServo2");
        armServo = hardwareMap.get(Servo.class,"armServo");
//        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
//        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
//        slideServo = hardwareMap.get(Servo.class, "slideServo");
//
//        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
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

//            leftFront.setPower(frontLeftPower);
//            leftBack.setPower(backLeftPower);
//            rightFront.setPower(frontRightPower);
//            rightBack.setPower(backRightPower);

            if (gamepad1.a || gamepad2.a){
                speed = intakeSpeed;
            }
            if (gamepad1.b || gamepad2.b){
                speed = intakeOutSpeed;
            }
            if (gamepad2.start || gamepad1.options) {
               speed = 0;
            }
            // Outake position
            if (gamepad2.y){
                armHeight = armIntakeHeight;
            }
            //Intake Position
            if (gamepad2.x){
                armHeight = armOuttakeHeight;
            }
            // Middle position
            if (gamepad2.right_stick_button){
                armHeight = armMidHeight;
            }

//            if (gamepad2.dpad_down){
//                elevatorMotor.setTargetPosition(startingHeight);
//                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevatorMotor.setPower (1);
//            }
//            if (gamepad2.dpad_up){
//                elevatorMotor.setTargetPosition(highBucketHeight);
//                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevatorMotor.setPower (1);
//            }
//
//            if (gamepad2.dpad_right){
//                elevatorMotor.setTargetPosition(lowBucketHeight);
//                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevatorMotor.setPower (1);
//            }
//            if (gamepad2.left_bumper){
//                slidePosition = slidesIn;
//                armHeight = armOuttakeHeight;
//                bucketPosition = bucketIntake;
//            }
            if (gamepad2.right_bumper){
                slidePosition = slidesExtended;
                armHeight = armIntakeHeight;
                bucketPosition = bucketMid;
            }
            if (gamepad2.left_trigger > 0.5){
                bucketPosition = bucketIntake;
            }
            if (gamepad2.right_trigger > 0.5){
                bucketPosition = bucketDump;
            }
            if (gamepad2.left_stick_button){
                bucketPosition = bucketMid;
            }


            intakeServo1.setPower(speed);
            intakeServo2.setPower(-speed);
            armServo.setPosition(armHeight);
            slideServo.setPosition(slidePosition);
            bucketServo.setPosition(bucketPosition);


            telemetry.addData("Status", "Running");
            telemetry.addData("lift height",  elevatorMotor.getCurrentPosition());
            telemetry.update();

        }
    }
}
