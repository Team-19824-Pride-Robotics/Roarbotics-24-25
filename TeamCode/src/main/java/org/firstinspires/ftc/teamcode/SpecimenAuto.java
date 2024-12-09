package org.firstinspires.ftc.teamcode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "SpecimenAuto")
public class SpecimenAuto extends LinearOpMode {

    public static double in_speed = -1;
    public static double out_speed = 0.5;
    public static int lift_high_bucket = -4000;
    public static int lift_transfer = 0;
    public static int lift_spec_pickup = -250;
    public static int lift_spec_score = -700;
    public static double bucket_dump = 0.02;
    public static double bucket_transfer = 0.02;
    public static double bucket_mid = 0.3;
    public static double spec_arm_pickup = 0.03;
    public static double spec_arm_score = 0.6;
    public static double slides_extended = 0.82;
    public static double slides_transfer = 0.85;
    public static double slides_mid = 0.85;
    public static double arm_down = 0.03;
    public static double arm_transfer = 0.63;
    public static double pickup_speed = 5;
    public static double lift_time = 1;

    public static double x0 = 80;
    public static double x1 = 50;
    public static double x2 = 22;
    public static double y2 = -60;
    public static double x3 = 120;
    public static double x4 = 7;
    public static double y3 = -47;

    public static double x5 = 0;
    public static double x6 = 12;
    public static double y4 = -4;

    public static double x7 = 17;
    public static double y5 = -37;


    public class Intake {

        private CRServo intakeServo1;
        private CRServo intakeServo2;
        private Servo armServo;
        private Servo slideServo;

        public Intake(HardwareMap hardwareMap) {

            armServo = hardwareMap.get(Servo.class,"armServo");
            slideServo = hardwareMap.get(Servo.class, "slideServo");
            intakeServo1 = hardwareMap.get(CRServo.class, "intakeServo1");
            intakeServo2 = hardwareMap.get(CRServo.class, "intakeServo2");

        }

        public class ArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(arm_down);
                return false;
            }
        }
        public Action armDown() {
            return new ArmDown();
        }

        public class ArmTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(arm_transfer);
                return false;
            }
        }
        public Action armTransfer() {
            return new ArmTransfer();
        }

        public class SlidesExtend implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideServo.setPosition(slides_extended);
                return false;
            }
        }
        public Action slidesExtend() {
            return new SlidesExtend();
        }

        public class SlidesTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideServo.setPosition(slides_transfer);
                return false;
            }
        }
        public Action slidesTransfer() {
            return new SlidesTransfer();
        }

        public class SlidesMid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideServo.setPosition(slides_mid);
                return false;
            }
        }
        public Action slidesMid() {
            return new SlidesMid();
        }

        public class SpinFast implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeServo1.setPower(in_speed);
                intakeServo2.setPower(-in_speed);
                return false;
            }
        }
        public Action spinFast() {
            return new SpinFast();
        }

        public class SpinSlow implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeServo1.setPower(out_speed);
                intakeServo2.setPower(-out_speed);
                return false;
            }
        }
        public Action spinSlow() {
            return new SpinSlow();
        }

        public class NoSpin implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeServo1.setPower(0);
                intakeServo2.setPower(0);
                return false;
            }
        }
        public Action noSpin() {
            return new NoSpin();
        }

    }


    public class Lift {

        private DcMotorEx liftMotor1;
        private DcMotorEx liftMotor2;
        private Servo specArmServo;
        private Servo bucketServo;

        public Lift(HardwareMap hardwareMap) {

            liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
            liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
            liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor2.setDirection(DcMotorEx.Direction.REVERSE);

            specArmServo = hardwareMap.get(Servo.class,"specArmServo");
            bucketServo = hardwareMap.get(Servo.class, "bucketServo");

        }

        public class BasketHeight implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftMotor1.setTargetPosition(lift_high_bucket);
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor2.setTargetPosition(lift_high_bucket);
                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor1.setPower(1);
                liftMotor2.setPower(1);
                return false;
            }
        }
        public Action basketHeight() {
            return new BasketHeight();
        }

        public class TransferHeight implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftMotor1.setTargetPosition(lift_transfer);
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor2.setTargetPosition(lift_transfer);
                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor1.setPower(1);
                liftMotor2.setPower(1);
                return false;
            }
        }
        public Action transferHeight() {
            return new TransferHeight();
        }

        public class SpecimenPickupHeight implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftMotor1.setTargetPosition(lift_spec_pickup);
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor2.setTargetPosition(lift_spec_pickup);
                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor1.setPower(1);
                liftMotor2.setPower(1);
                return false;
            }
        }
        public Action specimenPickupHeight() {
            return new SpecimenPickupHeight();
        }

        public class SpecimenScoreHeight implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftMotor1.setTargetPosition(lift_spec_score);
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor2.setTargetPosition(lift_spec_score);
                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor1.setPower(1);
                liftMotor2.setPower(1);
                return false;
            }
        }
        public Action specimenScoreHeight() {
            return new SpecimenScoreHeight();
        }

        public class BucketTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucketServo.setPosition(bucket_transfer);
                return false;
            }
        }
        public Action bucketTransfer() {
            return new BucketTransfer();
        }

        public class BucketDump implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucketServo.setPosition(bucket_dump);
                return false;
            }
        }
        public Action bucketDump() {
            return new BucketDump();
        }

        public class BucketMid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucketServo.setPosition(bucket_mid);
                return false;
            }
        }
        public Action bucketMid() {
            return new BucketMid();
        }

        public class SpecArmPickup implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specArmServo.setPosition(spec_arm_pickup);
                return false;
            }
        }
        public Action specArmPickup() {
            return new SpecArmPickup();
        }

        public class SpecArmScore implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specArmServo.setPosition(spec_arm_score);
                return false;
            }
        }
        public Action specArmScore() {
            return new SpecArmScore();
        }

    }



    @Override
    public void runOpMode() throws InterruptedException {


        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make an Intake instance
        Intake intake = new Intake(hardwareMap);
        // make a Lift instance
        Lift lift = new Lift(hardwareMap);


        TrajectoryActionBuilder segment1;
        TrajectoryActionBuilder segment2;
        TrajectoryActionBuilder segment2_5;
        TrajectoryActionBuilder segment3;

        Action segment4;
        Action segment5;
        Action segment6;
        Action segment7;
        Action segment8;

        //segment 1 - drives up to the sub and scores the preload
        // parallel with lift to score height
        segment1 = drive.actionBuilder(initialPose)
                .lineToXConstantHeading(x0);

        Action seg1 = segment1.build();

        //segment 2 - backs off the sub

        segment2 = segment1.endTrajectory().fresh()
                .lineToXConstantHeading(x1);


        Action seg2 = segment2.build();

        //segment 2.5 - strafes right to clear the sub
        // parallel with lift to pickup position

        segment2_5 = segment2.endTrajectory().fresh()
                  .strafeTo(new Vector2d(x1, y2));

        Action seg2_5 = segment2_5.build();

        //segment 3 - moves on a diagonal to get behind the sample
        segment3 = segment2_5.endTrajectory().fresh()
                .setTangent(0)
                .lineToX(x3);

        Action seg3 = segment3.build();

        //segment 4 - spline path with a 180 built in, gets in position to push
        segment4 = drive.actionBuilder(drive.pose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(20, 0, Math.toRadians(0)), Math.toRadians(0))
                .build();

        //segment 5 - push two samples into the zone
        segment5 = drive.actionBuilder(drive.pose)
                .lineToX(x4)
                .lineToX(x3)
                .strafeTo(new Vector2d(x3, y3))
                .setTangent(0)
                .lineToX(x4)
                .build();

        //segment 6 - slowly! to pick up the specimen
        segment6 = drive.actionBuilder(drive.pose)
                .lineToX(x5, new TranslationalVelConstraint(pickup_speed))
                .build();

        //segment 7 - spline path back to the sub with a 180
        //parallel with lift to scoring position
        segment7 = drive.actionBuilder(drive.pose)
                .lineToX(x6)
                .splineToLinearHeading(new Pose2d(x0, y4, Math.toRadians(0)), Math.toRadians(0))
                .build();

        //segment 8 - spline path back to the zone with a 180
        // parallel with lift to pickup position
        segment8 = drive.actionBuilder(drive.pose)
                .lineToX(x7)
                .splineToLinearHeading(new Pose2d(x4, y5, Math.toRadians(0)), Math.toRadians(0))
                .build();


        waitForStart();




        if (isStopRequested()) return;


        Actions.runBlocking(new SequentialAction(

                new ParallelAction(
                        seg1,
                        lift.specimenScoreHeight(),
                        lift.specArmScore()
                ),

                seg2,

                new ParallelAction(
                        seg2_5,
                        lift.specimenPickupHeight(),
                        lift.specArmPickup()
                ),

                seg3
/*
                segment4,

                segment5,

                segment6,

                lift.specimenScoreHeight(),  //this takes the specimen off the wall

                new SleepAction(lift_time),  //it needs time to go up before driving away

                new ParallelAction(
                        segment7,
                        lift.specArmScore()
                ),

                new ParallelAction(
                        segment8,
                        lift.specimenPickupHeight(),
                        lift.specArmPickup()
                ),

                segment6,

                lift.specimenScoreHeight(),  //this takes the specimen off the wall

                new SleepAction(lift_time),  //it needs time to go up before driving away

                new ParallelAction(
                        segment7,
                        lift.specArmScore()
                ),

                new ParallelAction(
                        segment8,
                        lift.specimenPickupHeight(),
                        lift.specArmPickup()
                ),

                segment6,

                lift.specimenScoreHeight(),  //this takes the specimen off the wall

                new SleepAction(lift_time),  //it needs time to go up before driving away

                new ParallelAction(
                        segment7,
                        lift.specArmScore()
                )

 */

                ));


    }
}
