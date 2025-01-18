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
    public static int lift_spec_pickup = -400;
    public static int lift_spec_score = -850;
    public static double bucket_dump = 0.65;
    public static double bucket_transfer = 0.05;
    public static double bucket_mid = 0.38;
    public static double spec_arm_pickup = 0.4;
    public static double spec_arm_score = 0.93;
    public static int slides_extended = -210;
    public static int slides_transfer = -50;
    public static int slides_mid = -100;
    public static double arm_down = 0.08;
    public static double arm_transfer = 0.8;
    public static double pickup_speed = 25;
    public static double lift_time = 1;
    public static double spec_arm_park = 0.8;

    public static double x0 = 70;
    public static double x1 = 50;
    public static double x2 = 22;
    public static double y2 = -60;
    public static double x3 = 120;
    public static double x8 = 120;
    public static double y6 = -85;
    public static double x4 = 20;
    public static double y3 = -110;

    public static double x5 = -7;
    public static double x6 = 12;
    public static double y4 = -4;

    public static double x7 = 17;
    public static double y5 = -37;
    public static double x9 = 40;
    public static double y9 = 0;
    public static double y11 = 20;
    public static double x10 = 30;
    public static double y12 = 0;
    public static double y13 = 110;
    public static double x11 = 120;
    public static double x12 = 20;
    public static double y14 = 105;
    public static double y15 = -66;
    public static double x13 = 25;
    public static double y16 = 80;
    public static double y17 = -110;
    public static double y18 = -110;
    public static double x14 = 80;



    public class Intake {

        private CRServo intakeServo1;
        private CRServo intakeServo2;
        private Servo armServo;
        private DcMotorEx slidesMotor;

        public Intake(HardwareMap hardwareMap) {

            armServo = hardwareMap.get(Servo.class,"armServo");
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

        public class SpecArmPark implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specArmServo.setPosition(spec_arm_park);
                return false;
            }
        }


        public Action specArmPark() {
            return new SpecArmPark();
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
        TrajectoryActionBuilder segment4;
        TrajectoryActionBuilder segment5;
        TrajectoryActionBuilder segment6;
        TrajectoryActionBuilder segment7;
        TrajectoryActionBuilder segment7_5;
        TrajectoryActionBuilder segment7_6;
        TrajectoryActionBuilder segment8;
        TrajectoryActionBuilder segment8_5;
        TrajectoryActionBuilder segment9;
        TrajectoryActionBuilder segment10;
        TrajectoryActionBuilder segment11;
        TrajectoryActionBuilder segment12;
        TrajectoryActionBuilder segment13;
        TrajectoryActionBuilder segment14;
        //segment 1 - drives up to the sub and scores the preload
        // parallel with lift to score height
        segment1 = drive.actionBuilder(initialPose)
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(x0, 0));

        Action seg1 = segment1.build();

        //segment 2 - backs off the sub

        segment2 = segment1.endTrajectory().fresh()
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(x1, 0));

        Action seg2 = segment2.build();

        //segment 2.5 - strafes right to clear the sub
        // parallel with lift to pickup position

        segment2_5 = segment2.endTrajectory().fresh()
                  .strafeTo(new Vector2d(x14, y2));

        Action seg2_5 = segment2_5.build();

        //segment 3 - moves on a diagonal to get behind the sample
        segment3 = segment2_5.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(x3, y6), Math.toRadians(0));

        Action seg3 = segment3.build();

        //segment 4 - push a sample into the obs zone
        segment4 = segment3.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(x4, y6));

        Action seg4 = segment4.build();

        //segment 5 - push two samples into the zone
        segment5 = segment4.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(x13, y15));

        Action seg5 = segment5.build();
//
//        segment5_5 = segment5.endTrajectory().fresh()
//                .strafeToConstantHeading(new Vector2d(x4, y6));
//
//        Action seg5_5 = segment5_5.build();

        //segment 6 - slowly! to pick up the specimen
        segment6 = segment5.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(x5,y15), new TranslationalVelConstraint(pickup_speed));

        Action seg6 = segment6.build();

        //segment 7 - strafe back to the sub with a 180
        //parallel with lift to scoring position
        segment7 = segment6.endTrajectory().fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(x9, y9), Math.toRadians(180));

        Action seg7 = segment7.build();

        //segment 7.5 - scoring second specimen
        segment7_5 = segment7.endTrajectory().fresh()
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(x14, y11));

        Action seg7_5 = segment7_5.build();

        segment7_6 = segment7_5.endTrajectory().fresh()
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(x1, 0));

        Action seg7_6 = segment7_6.build();

        //segment 8 - strafe path back to the zone with a 180
        // parallel with lift to pickup position
        segment8 = segment7_6.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x4, y6), Math.toRadians(0));

        Action seg8 = segment8.build();

//segment 8_5 - slowly! to pick up the specimen
        segment8_5 = segment8.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(x5,y6), new TranslationalVelConstraint(pickup_speed));

        Action seg8_5 = segment8_5.build();
//
        segment9 = segment8_5.endTrajectory().fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(x9, y9), Math.toRadians(180));

        Action seg9 = segment9.build();
//Turn Around and go to put specimen on the bar
        segment10 = segment9.endTrajectory().fresh()

                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(x14, y11));

        Action seg10 = segment10.build();
//goes back and to the right in anticipation of pushing the block
        segment11 = segment10.endTrajectory().fresh()

                .strafeToConstantHeading(new Vector2d(x10, y12));

        Action seg11 = segment11.build();

        segment12 = segment11.endTrajectory().fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(x11, y17), Math.toRadians(0));

        Action seg12 = segment12.build();

        segment13 = segment12.endTrajectory().fresh()

                .strafeToConstantHeading(new Vector2d(x12, y18));

        Action seg13 = segment13.build();

        segment14 = segment13.endTrajectory().fresh()

                .strafeToConstantHeading(new Vector2d(x12, y18));

        Action seg14 = segment14.build();


        waitForStart();


        if (isStopRequested()) return;


        Actions.runBlocking(new SequentialAction(

                new ParallelAction(
                        seg1,
                        lift.specimenScoreHeight(),
                        lift.specArmScore(),
                        intake.armDown()
                ),

                new ParallelAction(
                        seg2,
                        lift.specimenPickupHeight()
                ),

                new ParallelAction(
                        seg2_5,
                        lift.specArmPickup()
                ),

                seg3,

                seg4,

                seg5,

                seg6,

                lift.specimenScoreHeight(),  //this takes the specimen off the wall

//                new SleepAction(lift_time),  //it needs time to go up before driving away

                new ParallelAction(
                        seg7,
                        lift.specArmScore()
                ),

                seg7_5,
                new ParallelAction(
                        seg7_6,
                        lift.specimenPickupHeight()
                ),

                new ParallelAction(
                        seg8,
                        lift.specArmPickup()
                ),

                seg8_5,

                lift.specimenScoreHeight(),  //this takes the specimen off the wall

//                new SleepAction(lift_time),  //it needs time to go up before driving away

                new ParallelAction(
                        seg9,
                        lift.specArmScore()
                ),

                new ParallelAction(
                        seg10,
                        lift.specArmScore()
                ),

                new ParallelAction(
                       seg11,
                        lift.transferHeight()
                ),
                lift.specArmPickup(),

                seg12,

                seg13,

                new ParallelAction(

                        seg14,

                        lift.specArmPark())






                ));


    }
}
