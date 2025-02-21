package org.firstinspires.ftc.teamcode;

// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "SampleAuto")
public class SampleAuto extends LinearOpMode {

    public static double in_speed = -1;
    public static double out_speed = 1;
    public static int lift_high_bucket = -4000;
    public static int lift_transfer = 0;
    public static int lift_spec_pickup = -250;
    public static int lift_spec_score = -700;
    public static double bucket_dump = 0.85;
    public static double bucket_transfer = 0.1;
    public static double bucket_mid = 0.3;
    public static double spec_arm_pickup = 0.4;
    public static double spec_arm_score = 0.93;
    public static int slides_extended = -350;
    public static int slides_transfer = -50;
    public static int slides_mid = -200;
    public static int slides_in = 0;
    public static double arm_down = 0.67;
    public static double arm_transfer = 0.1;
    public static double pickup_speed = 25;
    public static double lift_time = 1;
    public static double spec_arm_park = 0.8;
    public static double intake_time = 0.75;
    public static double block_open = 0;
    public static double block_closed = 0.4;
    public static double arm_mid = 0.4;
    public static double arm_mid2 = 0.2;

    public static double xs = 10;
    public static double ys = 60;
    public static double hs = 315;
    public static double x1 = 40;
    public static double y1 = 19;
    public static double h1 = 0;
    public static double x2 = 10;
    public static double y2 = 40;
    public static double h2 = 58;
    public static double x3 = 62;
    public static double y3 = 45;
    public static double h3 = 20;
    public static double xp = 104;
    public static double yp = 26;
    public static double hp = 90;
    public static double y4 = 34;


    public class Intake {

        private CRServo intakeServo1;
        private CRServo intakeServo2;
        private Servo armServo;
        private DcMotorEx slidesMotor;
        private Servo blockServo;

        public Intake(HardwareMap hardwareMap) {

            armServo = hardwareMap.get(Servo.class,"armServo");
            blockServo = hardwareMap.get(Servo.class, "blockServo");

            slidesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");
            slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        public class ArmMid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(arm_mid);
                return false;
            }
        }
        public Action armMid() {
            return new ArmMid();
        }

        public class ArmMid2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(arm_mid2);
                return false;
            }
        }
        public Action armMid2() {
            return new ArmMid();
        }



        public class UnBlock implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                blockServo.setPosition(block_open);
                return false;
            }
        }
        public Action unBlock() { return new UnBlock(); }





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
                slidesMotor.setTargetPosition(slides_extended);
                slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesMotor.setPower(1);
                return false;
            }
        }
        public Action slidesExtend() {
            return new SlidesExtend();
        }

        public class SlidesTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slidesMotor.setTargetPosition(slides_transfer);
                slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesMotor.setPower(1);
                return false;
            }
        }
        public Action slidesTransfer() {
            return new SlidesTransfer();
        }

        public class SlidesMid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slidesMotor.setTargetPosition(slides_mid);
                slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesMotor.setPower(1);
                return false;
            }
        }

        public Action slidesMid() {
            return new SlidesMid();
        }



        public class SlidesIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slidesMotor.setTargetPosition(slides_in);
                slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesMotor.setPower(1);
                return false;
            }
        }

        public Action slidesIn() {
            return new SlidesIn();
        }





        public class SpinFast implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeServo1.setPower(-in_speed);
                intakeServo2.setPower(in_speed);
                return false;
            }
        }
        public Action spinFast() {
            return new SpinFast();
        }

        public class SpinSlow implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeServo1.setPower(-out_speed);
                intakeServo2.setPower(out_speed);
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

        public class SlowSlidesDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sleep(500);
                liftMotor1.setTargetPosition(lift_transfer);
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor2.setTargetPosition(lift_transfer);
                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor1.setPower(1);
                liftMotor2.setPower(1);
                return false;
            }
        }
        public Action slowSlidesDown() {
            return new SlowSlidesDown();
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

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make an Intake instance
        Intake intake = new Intake(hardwareMap);
        // make a Lift instance
        Lift lift = new Lift(hardwareMap);


        TrajectoryActionBuilder segment1;
        TrajectoryActionBuilder segment1_5;
        TrajectoryActionBuilder segment2;
        TrajectoryActionBuilder segment3;
        TrajectoryActionBuilder segment3_5;
        TrajectoryActionBuilder segment4;
        TrajectoryActionBuilder segment5;
        TrajectoryActionBuilder segment5_5;
        TrajectoryActionBuilder segment6;
        TrajectoryActionBuilder segment7;
        TrajectoryActionBuilder segment7_5;
        TrajectoryActionBuilder segment8;

                //segment 1 - drives up to the basket and scores the preload
                // parallel with lift to score position, add wait time for bucket to score
                segment1 = drive.actionBuilder(initialPose)
                        .strafeToLinearHeading(new Vector2d(x2, y2), Math.toRadians(hs), new TranslationalVelConstraint(200));

                Action seg1 = segment1.build();

        segment1_5 = segment1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(xs, ys), Math.toRadians(hs), new TranslationalVelConstraint(200));

        Action seg1_5 = segment1_5.build();


                //segment 2 - get in position to grab second sample
                // parallel with lift and arm to pickup position, add wait time for pickup
                segment2 = segment1.endTrajectory().fresh()
                        .strafeToLinearHeading(new Vector2d(x1, y1), Math.toRadians(h1), new TranslationalVelConstraint(200));

                Action seg2 = segment2.build();
//
//
                //segment 3 - moves in position to score the sample
                // parallel with lift and arm to score position, add wait time for score
                segment3 = segment2.endTrajectory().fresh()
                        .strafeToLinearHeading(new Vector2d(x2, y2), Math.toRadians(hs), new TranslationalVelConstraint(200));

                Action seg3 = segment3.build();

        segment3_5 = segment3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(xs, ys), Math.toRadians(hs), new TranslationalVelConstraint(200));

        Action seg3_5 = segment3_5.build();




                //segment 4 - strafe over for the third sample
                // parallel with lift and arm to pickup position, add wait time for pickupSERAv
                segment4 = segment3_5.endTrajectory().fresh()
                        .strafeToLinearHeading(new Vector2d(x1, y3), Math.toRadians(h1), new TranslationalVelConstraint(200));



                Action seg4 = segment4.build();


                //segment 5 - moves in position to score the sample
                // parallel with lift and arm to score position, add wait time for score
                segment5 = segment4.endTrajectory().fresh()
                        .strafeToLinearHeading(new Vector2d(x2, y2), Math.toRadians(hs), new TranslationalVelConstraint(200));

                Action seg5 = segment5.build();

        segment5_5 = segment5.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(xs, ys), Math.toRadians(hs), new TranslationalVelConstraint(200));

        Action seg5_5 = segment5_5.build();

                //segment 6 - strafe to same position, new angle
                // parallel with lift and arm to pickup position, add wait time for pickup
                segment6 = segment5_5.endTrajectory().fresh()
                        .strafeToLinearHeading(new Vector2d(x3, y4), Math.toRadians(h2), new TranslationalVelConstraint(200));

                Action seg6 = segment6.build();


                //segment 5 - moves in position to score the sample
                // parallel with lift and arm to score position, add wait time for score
                segment7 = segment6.endTrajectory().fresh()
                        .strafeToLinearHeading(new Vector2d(x2, y2), Math.toRadians(hs), new TranslationalVelConstraint(200));

                Action seg7 = segment7.build();

        segment7_5 = segment7.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(xs, ys), Math.toRadians(hs), new TranslationalVelConstraint(200));

        Action seg7_5 = segment7_5.build();
//
//                //segment 8 - park in ascent zone
//                segment8 = segment7.endTrajectory().fresh()
//                        .strafeToLinearHeading(new Vector2d(xp, yp), Math.toRadians(hp));
// sillu willy prank
//                Action seg8 = segment8.build();


        intake.unBlock();
        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(new SequentialAction(

                //put the slides out so the bucket can go up
                intake.unBlock(),
                intake.slidesMid(),
                intake.armMid(),
                lift.bucketMid(),
                new SleepAction(1),

                new ParallelAction(
                        seg1,
                        lift.basketHeight(),
                        lift.bucketMid()
                ),


                seg1_5,
                //score the preloaded sample


                        lift.bucketDump(),



                new SleepAction(1),

                new ParallelAction(
                        seg2,
                        lift.bucketTransfer(),
                        intake.slidesExtend(),
                        lift.slowSlidesDown()
                ),


                //intake the next sample
                intake.armMid(),
                intake.spinFast(),
                new SleepAction(0.5),
                intake.armDown(),
                new SleepAction(intake_time),
                intake.noSpin(),
                intake.armTransfer(),
                intake.slidesMid(),
                new SleepAction(0.5),
                new ParallelAction(
                        seg3,
                        lift.basketHeight(),
                        lift.bucketMid()
                ),

                intake.spinSlow(),
                    new SleepAction(0.5),
                seg3_5,

                lift.bucketDump(),
                new SleepAction(0.5),
                new ParallelAction(
                        seg4,
                        lift.slowSlidesDown(),
                        intake.slidesExtend(),
                        lift.bucketTransfer()
                ),


                //intake the next sample
                intake.armMid(),
                intake.spinFast(),
                new SleepAction(0.5),
                intake.armDown(),
                new SleepAction(intake_time),
                intake.noSpin(),
                intake.armTransfer(),
                intake.slidesMid(),
                new SleepAction(0.5),

                new ParallelAction(
                        seg5,
                        lift.basketHeight(),
                        lift.bucketMid()
                ),

                intake.spinSlow(),
                new SleepAction(0.5),
                seg5_5,

                lift.bucketDump(),
                new SleepAction(0.5),
        new ParallelAction(
                intake.armMid2(),
                seg6,
                lift.bucketTransfer(),
                intake.slidesExtend(),
                lift.slowSlidesDown()
        ),

                lift.transferHeight(),
                //intake the next sample
                intake.spinFast(),
                new SleepAction(0.5),
                intake.armDown(),
                new SleepAction(intake_time),
                intake.noSpin(),
                intake.armTransfer(),
                intake.slidesMid(),
                new SleepAction(0.5),
                new ParallelAction(
                        seg7,
                        lift.basketHeight(),
                        lift.bucketMid()
                ),


                intake.spinSlow(),
                new SleepAction(0.5),
                seg7_5,

                lift.bucketDump(),
                new SleepAction(0.5)
//
//                //score the sample
////                intake.slidesMid(),
////                new SleepAction(0.5),
//                lift.basketHeight(),
//                lift.bucketMid(),
//                new SleepAction(2),
//                lift.bucketDump(),
//                new SleepAction(0.5),
//
//                new ParallelAction(
//                        seg8,
//                        lift.transferHeight(),
//                        intake.slidesIn()
//                )



                ));


    }
}
