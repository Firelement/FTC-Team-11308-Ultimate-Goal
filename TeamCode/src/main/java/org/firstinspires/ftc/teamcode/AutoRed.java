package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.sql.Driver;
import java.util.List;

@Autonomous(name="Auto Red", group="Pushbot")
public class AutoRed extends LinearOpMode {

    /** Variables for ring detection */
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            " AaDyemX/////AAABmTAtC2ONB0wHkUhRXShDQ5YMqzoOqj1zpW+eTxr9Vmb+zLP/5S8iHdsIsvHZDQDOgEQ8Q3j+Ke6cIggKMyK/1dGYnFe3e9oa/E/VCSqAwxd5EZZzKTEHnc+JATc5WBgX9zhdEFFZuuo1Xsn8Hpo93GCQC5n5q4uRhEAt7XvqRpj7qFT48aKhv2yHCraHMdrcD9NHXtr1CNpS53Qi9k/SrRvn9PdKL4taAey2C53xqvQ1j/+xh3Eh+3ORnMwaySnccNf145o9f5yv4fquBGYJfity4VIblSqAyg3VX/S/4aj8UmPe3T04Idl64z//OpS3ZXfozz/4Gk1qA9nW6twomt6e4kLrp3nLLiM6NOQGC1/e";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    //Wobble Arm
    private final double LIFT_POWER = 0.8;
    private final double CLOSED_LEFT_SERVO = 0.0;
    private final double CLOSED_RIGHT_SERVO = 0.6;
    private final double OPEN_LEFT_SERVO = 0.6;
    private final double OPEN_RIGHT_SERVO = 0.0;

    //Flywheel
    private final double FLYWHEEL_POWER = 0.55;//This value may need additional logic if we need to vary the power.
    private final double FLYWHEEL_POWERSHOT = 0.45;

    //Intake
    private final double INTAKE_POWER1 = 0.75;// This value has not been tested.
    private final double INTAKE_POWER2 = 0.5;
    private final double RING_STOPPER_POWER = -1.0;// This value has not been tested yet either;
    private final double INTAKE_RELEASE_POWER = 1.0;

    //Timing
    private ElapsedTime runtime = new ElapsedTime();

    //Drive
    private static final double COUNTS_PER_INCH = 1400; //Counts on odometry wheel per inch of distance
    private static final double COUNTS_PER_DEGREE = 135; //Counts on odometry wheel per inch of distance

    private static final double DRIVE_SPEED = 0.5; //Speed of wheel
    private static final double STRAFE_SPEED = 0.55;
    private static double leftFrontPower;
    private static double leftRearPower;
    private static double rightFrontPower;
    private static double rightRearPower;
    private static double DRIVE_ADJUSTMENT = 0; //Variable for power adjustment
    private static final double POWER_ADJUSTMENT_CONSTANT = 10000.0;    //Variable that dictates how much power adjustment will be added based on encoder difference in odometry
    private static final double POWER_ADJUSTMENT_CONSTANT_STRAFING = 3000.0;

    //Other variables
    private int rings = 0;    //Ring count variable for pathing
    private int heldRings = 3; // Amount of held rings in robot
    private int detectionTries = 10; // Amount of held rings in robot

    /** Motor declarations */
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftRearDrive;
    private DcMotor rightRearDrive;
    private DcMotor wobbleLifter;
    private DcMotor intake1;
    private DcMotor intake2;
    private DcMotor flyWheel;

    /** Servo declarations */
    private CRServo ringStopper;
    private CRServo intakeRelease;
    private Servo rightServo;
    private Servo leftServo;

    //IMU
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double  globalAngle= .30;

    // Color sensor
    NormalizedColorSensor colorSensor;
    private final double WOBBLE_GOAL_DIST = 7.0;

    @Override
    public void runOpMode() {

        // Activate TensorFlow Object Detection before we wait for the start command
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }

        //Initialize the Motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        wobbleLifter = hardwareMap.get(DcMotor.class, "wobble_lifter");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        flyWheel = hardwareMap.get(DcMotor.class, "fly_wheel");

        //Initialize Servos
        //ringStopper = hardwareMap.get(Servo.class,"ring_stopper");
        intakeRelease = hardwareMap.get(CRServo.class, "intake_release");
        ringStopper = hardwareMap.get(CRServo.class, "ring_stopper");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        //Initialize Color sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        //Set motor directions and modes
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wobbleLifter.setDirection(DcMotor.Direction.FORWARD);
        intake1.setDirection(DcMotor.Direction.REVERSE);
        intake2.setDirection(DcMotor.Direction.FORWARD);
        flyWheel.setDirection(DcMotor.Direction.REVERSE);
        flyWheel.setPower(0.0);

        //Initialize the Servo positions
        leftServo.setPosition(CLOSED_LEFT_SERVO);
        rightServo.setPosition(CLOSED_RIGHT_SERVO);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }


        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addLine("Ready To Start A");
        telemetry.update();
        waitForStart();

            /* Autonomous section */

            //Turn on flywheel to allow spinup time
            flyWheel.setPower(FLYWHEEL_POWERSHOT);
            //Drive to see rings - Encoder Drive
            encoderDriveY(DRIVE_SPEED, 31, 5);
            encoderDriveX(STRAFE_SPEED, -5, 1);
            //Detect Ring Stack - Detect Rings
            detectRings();
            //Drive to power shot shooting location
            encoderDriveX(STRAFE_SPEED, -31, 5);
            //Release intake servo during movement
            intakeRelease.setPower(INTAKE_RELEASE_POWER);
            encoderDriveY(DRIVE_SPEED, 30, 5);
            intakeRelease.setPower(0);
            //Power Shots  - Shoot 1 ring 3 times
            //Raise fire blocker
            ringStopper.setPower(RING_STOPPER_POWER);
            sleep(500);
            ringStopper.setPower(0);
            encoderDriveX(STRAFE_SPEED, -1, 1);
            //Shoot ring 1
        shootRings(1);
        //Line up next shot
        encoderDriveX(STRAFE_SPEED, 6, 2);
        //Shoot ring 2
        shootRings(1);
        //Line up next shot
        encoderDriveX(STRAFE_SPEED, 6, 2);
        //Shoot final ring
        shootRings(1);

            //Determine where to place wobble goal and next autonomous steps
            if (rings == 0) {//0 Rings
                //Drive to wobble goal deposit area
                encoderDriveX(STRAFE_SPEED, 50, 5);
                encoderDriveY(DRIVE_SPEED, 7, 1);
                //encoderRotate(DRIVE_SPEED,90,3);
                sleep(100);
                resetAngle();

                rotate(-75, DRIVE_SPEED);
                //Drop wobble goal
                dropWobbleGoal();

                //encoderRotate(DRIVE_SPEED,110,5);
                rotate(-60, DRIVE_SPEED);

                //Turn towards wobble goal
                sleep(250);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                sleep(250);

                //Move to pick up wobble goal

                pickUpWobbleGoal(DRIVE_SPEED, 50,5);
                encoderDriveY(DRIVE_SPEED, -50, 5);
                //encoderRotate(DRIVE_SPEED, -50, 2);
                rotate(60, DRIVE_SPEED);
                dropWobbleGoal();
                //Parked on line

            } else if (rings == 1) {//1 Ring
                //Drive to wobble goal deposit area
                encoderDriveX(STRAFE_SPEED, 47, 3);
                encoderDriveY(DRIVE_SPEED, 15, 3);
                //Drop wobble goal
                dropWobbleGoal();
                //Grab last ring for shooting
                encoderDriveX(STRAFE_SPEED, -4,1);
                intakeWheelDrive(DRIVE_SPEED, -45, 6);

                heldRings = 1;
                //Drive to goal shooting location
                encoderDriveY(DRIVE_SPEED, 25, 6);
                intake1.setPower(INTAKE_POWER1);
                intake2.setPower(INTAKE_POWER2);
                flyWheel.setPower(FLYWHEEL_POWER);
                encoderDriveX(STRAFE_SPEED, -15,1);
                //Shoot ring
                shootRings(1);
                //Park on line
                encoderDriveY(DRIVE_SPEED, 10, 1);
            } else { //4 Rings
                //Drive to wobble goal deposit area
                encoderDriveX(STRAFE_SPEED, 82, 4);
                encoderDriveX(STRAFE_SPEED, -0.25, 1);
                encoderDriveY(DRIVE_SPEED, 48, 5);
                //Drop wobble goal
                dropWobbleGoal();
                //Grab last ring for shooting
                encoderDriveY(DRIVE_SPEED, -30, 3);
               // encoderDriveX(STRAFE_SPEED, -37, 2);
               /* Too little time, Possibly will add back if other sections sped up
                intakeWheelDrive(DRIVE_SPEED, -35, 6);
                intakeWheelDrive(DRIVE_SPEED,-10,2);
                heldRings = 3;
                encoderDriveY(DRIVE_SPEED, 55, 6);


                 //Drive to goal shooting location
                encoderDriveY(DRIVE_SPEED, 35, 6);
                //Shoot rings
                shootRings(3);
                //intake last ring
                intakeWheelDrive(DRIVE_SPEED, -35, 6);
                heldRings = 1;
                //Drive to goal shooting location
                encoderDriveY(DRIVE_SPEED, 35, 6);
                //Shoot last ring
                shootRings(1);
                //Park on line
                encoderDriveY(DRIVE_SPEED, 5, 1);*/

            }
            //Turn flywheel off
        //    flyWheel.setPower(0);

        /** Autonomous Paths Notes */
        /*
        If: 4 rings
        Deposit wobble goal far - Open Wobble Claw
        Grab Rings - Intake Wheel Drive
        Shoot high goal - Shoot Rings
        Grab Last ring - Intake Wheel Drive
        Shoot high goal - Shoot Rings
        Park on line - Encoder Drive

        If: 1 ring
        Deposit wobble goal middle - Open Wobble Claw
        Grab Ring - Intake Wheel Drive
        Shoot high goal - Shoot Rings
        Park on line - Encoder Drive

        If: No Rings
        Deposit wobble goal closest - Open Wobble Claw
        Park on line - Encoder Drive
        */
       //     telemetry.addData("Path", "Complete");
         //   telemetry.update();


    }
        /** Drive forward and backwards with encoders */
        public void encoderDriveY(double speed, double inches, double timeoutS) {
        int newTarget;
        double leftSpeed;
        double rightSpeed;
            // Determine new target position, and pass to motor controller
            //This will be target position for both motors but will be determined by the left motor
            newTarget = leftFrontDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            // Ensure that the opmode is still active
        if (opModeIsActive()) {
             //reset runtime
                runtime.reset();
                if (inches <= 0) { //If inches is negative, robot moves backwards

                    while (opModeIsActive() && (leftFrontDrive.getCurrentPosition() >= newTarget || -rightFrontDrive.getCurrentPosition() >= newTarget && (runtime.seconds() < timeoutS))) {
                        // turn off motors and only turn on if needed to move
                        leftFrontDrive.setPower(0);
                        rightFrontDrive.setPower(0);
                        leftRearDrive.setPower(0);
                        rightRearDrive.setPower(0);

                        //Dynamically scale speed based on difference between encoders
                        DRIVE_ADJUSTMENT = (leftFrontDrive.getCurrentPosition() + rightFrontDrive.getCurrentPosition())/POWER_ADJUSTMENT_CONSTANT;
                        leftSpeed = speed + DRIVE_ADJUSTMENT;
                        rightSpeed = speed - DRIVE_ADJUSTMENT;

                        leftSpeed = Range.clip(leftSpeed, -1.0,1.0);
                        rightSpeed = Range.clip(rightSpeed, -1.0,1.0);

                        if (leftFrontDrive.getCurrentPosition() >= newTarget) {
                            leftFrontDrive.setPower(leftSpeed);
                            leftRearDrive.setPower(leftSpeed);
                        }
                        if (-rightFrontDrive.getCurrentPosition() >= newTarget) {
                            rightFrontDrive.setPower(rightSpeed);
                            rightRearDrive.setPower(rightSpeed);
                        }

                        telemetry.addData("Left Encoder:", leftFrontDrive.getCurrentPosition());
                        telemetry.addData("Right Encoder:",- rightFrontDrive.getCurrentPosition());
                        telemetry.addData("Target Position:", newTarget);
                        telemetry.addLine("Target Direction: Reverse");
                        telemetry.update();


                }
                } else { //If inches are positive, robot moves forward

                        while (opModeIsActive()&&( leftFrontDrive.getCurrentPosition() <= newTarget || -rightFrontDrive.getCurrentPosition() <= newTarget && (runtime.seconds() < timeoutS))) {
                            // turn off motors and only turn on if needed to move
                            leftFrontDrive.setPower(0);
                            rightFrontDrive.setPower(0);
                            leftRearDrive.setPower(0);
                            rightRearDrive.setPower(0);


                            //Dynamically scale speed based on difference between encoders
                            DRIVE_ADJUSTMENT = (leftFrontDrive.getCurrentPosition() + rightFrontDrive.getCurrentPosition())/POWER_ADJUSTMENT_CONSTANT;
                            leftSpeed = speed - DRIVE_ADJUSTMENT;
                            rightSpeed = speed + DRIVE_ADJUSTMENT;

                            leftSpeed = Range.clip(leftSpeed, -1.0,1.0);
                            rightSpeed = Range.clip(rightSpeed, -1.0,1.0);

                            if (leftFrontDrive.getCurrentPosition() <= newTarget) {
                                leftFrontDrive.setPower(-leftSpeed);
                                leftRearDrive.setPower(-leftSpeed);
                            }
                            if (-rightFrontDrive.getCurrentPosition() <= newTarget) {
                                rightFrontDrive.setPower(-rightSpeed);
                                rightRearDrive.setPower(-rightSpeed);
                            }

                            telemetry.addData("Left Encoder:", leftFrontDrive.getCurrentPosition());
                            telemetry.addData("Right Encoder:", -rightFrontDrive.getCurrentPosition());
                            telemetry.addData("Target Position:", newTarget);
                            telemetry.addLine("Target Direction: Forward");
                            telemetry.update();

                    }
                }
            // turn off motors
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftRearDrive.setPower(0);
            rightRearDrive.setPower(0);
            //      sleep(250);   // optional pause after each move
            }
        }

        public void encoderRotate( double speed, double degrees, double timeoutS){
            int newLTarget;
            int newRTarget;
            double leftSpeed;
            double rightSpeed;
            sleep(250);
            rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Determine new target position, and pass to motor controller
            //This will be target position for both motors but will be determined by the left motor
            newLTarget = leftFrontDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE );
            newRTarget = leftFrontDrive.getCurrentPosition() - (int) (degrees * COUNTS_PER_DEGREE);

            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                if(degrees > 0) {

                    while (opModeIsActive() && (leftFrontDrive.getCurrentPosition() <= newLTarget || -rightFrontDrive.getCurrentPosition() >= newRTarget )) {
                        leftSpeed = -speed;
                        rightSpeed = speed;

                        leftFrontDrive.setPower(leftSpeed);
                        leftRearDrive.setPower(leftSpeed);
                        rightFrontDrive.setPower(rightSpeed);
                        rightRearDrive.setPower(rightSpeed);
                        telemetry.addData("Left Encoder:", leftFrontDrive.getCurrentPosition());
                        telemetry.addData("Right Encoder:", -rightFrontDrive.getCurrentPosition());
                        telemetry.addData("Target Position L:", newLTarget);
                        telemetry.addData("Target Position R:", newRTarget);
                        telemetry.update();


                    }
                }else{
                    while (opModeIsActive() && (leftFrontDrive.getCurrentPosition() >= newLTarget || -rightFrontDrive.getCurrentPosition() <= newRTarget )) {
                        leftSpeed = speed;
                        rightSpeed = -speed;

                        leftFrontDrive.setPower(leftSpeed);
                        leftRearDrive.setPower(leftSpeed);
                        rightFrontDrive.setPower(rightSpeed);
                        rightRearDrive.setPower(rightSpeed);
                        telemetry.addData("Left Encoder:", leftFrontDrive.getCurrentPosition());
                        telemetry.addData("Right Encoder:",-rightFrontDrive.getCurrentPosition());
                        telemetry.addData("Target Position L:", newLTarget);
                        telemetry.addData("Target Position R:", newRTarget);
                        telemetry.update();

                    }
            }
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftRearDrive.setPower(0);
                rightRearDrive.setPower(0);

            }
        }

        /** Strafe with encoders*/
        public void encoderDriveX(double speed, double inches, double timeoutS) {
            int newTarget;

            // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = rightRearDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            //reset runtime
            runtime.reset();

            if (rightRearDrive.getCurrentPosition() > newTarget) { //If current position is larger than new target, robot goes right

                while (rightRearDrive.getCurrentPosition() >= newTarget  && (runtime.seconds() < timeoutS) && opModeIsActive()) {
                    leftFrontPower = speed;
                    rightFrontPower = -speed;
                    leftRearPower = -speed;
                    rightRearPower = speed;
                    /*
                    //Dynamically scale speed based on difference between encoders
                    DRIVE_ADJUSTMENT = (leftRearDrive.getCurrentPosition() + rightRearDrive.getCurrentPosition())/POWER_ADJUSTMENT_CONSTANT;

                    leftFrontPower += DRIVE_ADJUSTMENT;
                    leftRearPower += DRIVE_ADJUSTMENT;
                    rightFrontPower -= DRIVE_ADJUSTMENT;
                    rightRearPower -= DRIVE_ADJUSTMENT;



                    leftFrontPower = Range.clip(leftFrontPower, -1.0,1.0);
                    leftRearPower = Range.clip(leftRearPower, -1.0,1.0);
                    rightFrontPower = Range.clip(rightFrontPower, -1.0,1.0);
                    rightRearPower = Range.clip(rightRearPower, -1.0,1.0);
                    */
                    leftFrontDrive.setPower(leftFrontPower);
                    rightFrontDrive.setPower(rightFrontPower);
                    leftRearDrive.setPower(leftRearPower);
                    rightRearDrive.setPower(rightRearPower);

                   // telemetry.addData("Front Encoder:", -leftRearDrive.getCurrentPosition());
                    telemetry.addData("Back Encoder:", rightRearDrive.getCurrentPosition());
                    telemetry.addData("Target Position:", newTarget);
                    telemetry.addLine("Target Direction: Right");
                    telemetry.update();
                }
            } else { //If current position is smaller than new target, robot goes left
                while (rightRearDrive.getCurrentPosition() <= newTarget  && (runtime.seconds() < timeoutS) && opModeIsActive()) {
                    leftFrontPower = -speed;
                    rightFrontPower = speed;
                    leftRearPower = speed;
                    rightRearPower = -speed;
                    /*
                    //Dynamically scale speed based on difference between encoders
                    DRIVE_ADJUSTMENT = (leftRearDrive.getCurrentPosition() + rightRearDrive.getCurrentPosition())/POWER_ADJUSTMENT_CONSTANT;

                    leftFrontPower -= DRIVE_ADJUSTMENT;
                    leftRearPower -= DRIVE_ADJUSTMENT;
                    rightFrontPower += DRIVE_ADJUSTMENT;
                    rightRearPower += DRIVE_ADJUSTMENT;

                    leftFrontPower = Range.clip(leftFrontPower, -1.0,1.0);
                    leftRearPower = Range.clip(leftRearPower, -1.0,1.0);
                    rightFrontPower = Range.clip(rightFrontPower, -1.0,1.0);
                    rightRearPower = Range.clip(rightRearPower, -1.0,1.0);

                    */
                    leftFrontDrive.setPower(leftFrontPower);
                    rightFrontDrive.setPower(rightFrontPower);
                    leftRearDrive.setPower(leftRearPower);
                    rightRearDrive.setPower(rightRearPower);

                   // telemetry.addData("Front Encoder:", -leftRearDrive.getCurrentPosition());
                    telemetry.addData("Back Encoder:", rightRearDrive.getCurrentPosition());
                    telemetry.addData("Target Position:", newTarget);
                    telemetry.addLine("Target Direction: Left");
                    telemetry.update();
                }
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftRearDrive.setPower(0);
            rightRearDrive.setPower(0);
            sleep(250);   // optional pause after each move
            alignYEncoders();
        }
    }

        /** Straighten wheels according to Y encoders*/
        public void alignYEncoders(){
            if(leftFrontDrive.getCurrentPosition() > -rightFrontDrive.getCurrentPosition()){
                int newTarget = -rightFrontDrive.getCurrentPosition();
                while(leftFrontDrive.getCurrentPosition() > newTarget){
                    leftFrontDrive.setPower(DRIVE_SPEED/2);
                    leftRearDrive.setPower(DRIVE_SPEED/2);
                }
            }else if(leftFrontDrive.getCurrentPosition() < -rightFrontDrive.getCurrentPosition()){
                int newTarget = leftFrontDrive.getCurrentPosition();
                while(newTarget< -rightFrontDrive.getCurrentPosition()){
                    rightFrontDrive.setPower(DRIVE_SPEED/2);
                    rightRearDrive.setPower(DRIVE_SPEED/2);
                }
            }else{

            }
            leftFrontDrive.setPower(0);
            leftRearDrive.setPower(0);
            rightFrontDrive.setPower(0);

            rightRearDrive.setPower(0);
          //  sleep(150);
        }
        /** Driving  with intake on */
        public void intakeWheelDrive(double speed, double inches, double timeoutS){
            //Intake wheels on
            intake1.setPower(INTAKE_POWER1);
            intake2.setPower(INTAKE_POWER2);
            //Drive specified distance
            encoderDriveY(speed,inches,timeoutS);
            ringStopper.setPower(1);
            sleep(500);
            ringStopper.setPower(0);
            //Possible pause to allow rings to be held
            sleep(600);
            //Intake wheels off
            intake1.setPower(0);
            intake2.setPower(0);
        }

        /** Detect the amount of rings*/
        public void detectRings(){
            //Webcam Detection changes rings to number of rings- defaults to 0
            sleep(500);
            telemetry.addLine("Detection Tried");
            if (detectionTries>=0) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null ) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        if(recognition.getLabel().equalsIgnoreCase( "quad")){
                            rings = 4;
                            telemetry.addLine("Quad");
                            telemetry.update();

                        }else if(recognition.getLabel().equalsIgnoreCase("single")){
                            rings = 1;
                            telemetry.addLine("Single");
                            telemetry.update();

                        }else{
                            rings = 0;
                            detectionTries--;
                            sleep(100);
                            telemetry.update();
                            detectRings();
                        }
                    }
                }else {
                    rings = 0;
                    detectionTries--;
                    sleep(100);
                    telemetry.update();
                    detectRings();
                }

            }
            telemetry.update();
        }

        /** Shoot rings*/
        public void shootRings(int amountToShoot){
            //Launch rings for specific amount of time, according to amount to shoot
            intake1.setPower(INTAKE_POWER1);
            intake2.setPower(INTAKE_POWER2);
            ringStopper.setPower(0);
            for(int i = amountToShoot; i >0; i--){
                if(heldRings==1) {// if last ring
                    //Rotate fire blocker to launch last ring
                    ringStopper.setPower(RING_STOPPER_POWER);
                    sleep(1500);
                }else{// if not last ring
                    //Shoot ring by continuing to run intake
                    ringStopper.setPower(0);
                    sleep(700);
                }
                heldRings--;
            }
            intake1.setPower(0);
            intake2.setPower(0);
            ringStopper.setPower(0);
        }

        /** Drop the wobble goal*/
        public void dropWobbleGoal(){
            wobbleLifter.setPower(LIFT_POWER);
            sleep(1000);
            leftServo.setPosition(OPEN_LEFT_SERVO);
            rightServo.setPosition(OPEN_RIGHT_SERVO);
            sleep(100);
            wobbleLifter.setPower(-LIFT_POWER);
            sleep(800);
            wobbleLifter.setPower(0);
    }

        public void pickUpWobbleGoal(double speed, double inches, double timeoutS){
            int newTarget;
            double leftSpeed;
            double rightSpeed;
            leftServo.setPosition(OPEN_LEFT_SERVO);
            rightServo.setPosition(OPEN_RIGHT_SERVO);
            wobbleLifter.setPower(LIFT_POWER);
            sleep(800);
            wobbleLifter.setPower(0);
            // Determine new target position, and pass to motor controller
            //This will be target position for both motors but will be determined by the left motor
            newTarget = leftFrontDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                //reset runtime
                runtime.reset();
                if (inches <= 0) { //If inches is negative, robot moves backwards

                    while (opModeIsActive() && (leftFrontDrive.getCurrentPosition() >= newTarget || -rightFrontDrive.getCurrentPosition() >= newTarget && (runtime.seconds() < timeoutS))) {
                        // turn off motors and only turn on if needed to move
                        leftFrontDrive.setPower(0);
                        rightFrontDrive.setPower(0);
                        leftRearDrive.setPower(0);
                        rightRearDrive.setPower(0);

                        //Dynamically scale speed based on difference between encoders
                        DRIVE_ADJUSTMENT = (leftFrontDrive.getCurrentPosition() + rightFrontDrive.getCurrentPosition())/POWER_ADJUSTMENT_CONSTANT;
                        leftSpeed = speed + DRIVE_ADJUSTMENT;
                        rightSpeed = speed - DRIVE_ADJUSTMENT;

                        leftSpeed = Range.clip(leftSpeed, -1.0,1.0);
                        rightSpeed = Range.clip(rightSpeed, -1.0,1.0);

                        if (leftFrontDrive.getCurrentPosition() >= newTarget) {
                            leftFrontDrive.setPower(leftSpeed);
                            leftRearDrive.setPower(leftSpeed);
                        }
                        if (-rightFrontDrive.getCurrentPosition() >= newTarget) {
                            rightFrontDrive.setPower(rightSpeed);
                            rightRearDrive.setPower(rightSpeed);
                        }
                        if(((DistanceSensor)colorSensor).getDistance(DistanceUnit.CM) <= WOBBLE_GOAL_DIST ){
                            leftServo.setPosition(CLOSED_LEFT_SERVO);
                            rightServo.setPosition(CLOSED_RIGHT_SERVO);
                        }

                        telemetry.addData("Left Encoder:", leftFrontDrive.getCurrentPosition());
                        telemetry.addData("Right Encoder:",- rightFrontDrive.getCurrentPosition());
                        telemetry.addData("Target Position:", newTarget);
                        telemetry.addLine("Target Direction: Reverse");
                        telemetry.update();


                    }
                } else { //If inches are positive, robot moves forward

                    while (opModeIsActive()&&( leftFrontDrive.getCurrentPosition() <= newTarget || -rightFrontDrive.getCurrentPosition() <= newTarget && (runtime.seconds() < timeoutS))) {
                        // turn off motors and only turn on if needed to move
                        leftFrontDrive.setPower(0);
                        rightFrontDrive.setPower(0);
                        leftRearDrive.setPower(0);
                        rightRearDrive.setPower(0);


                        //Dynamically scale speed based on difference between encoders
                        DRIVE_ADJUSTMENT = (leftFrontDrive.getCurrentPosition() + rightFrontDrive.getCurrentPosition())/POWER_ADJUSTMENT_CONSTANT;
                        leftSpeed = speed - DRIVE_ADJUSTMENT;
                        rightSpeed = speed + DRIVE_ADJUSTMENT;

                        leftSpeed = Range.clip(leftSpeed, -1.0,1.0);
                        rightSpeed = Range.clip(rightSpeed, -1.0,1.0);

                        if (leftFrontDrive.getCurrentPosition() <= newTarget) {
                            leftFrontDrive.setPower(-leftSpeed);
                            leftRearDrive.setPower(-leftSpeed);
                        }
                        if (-rightFrontDrive.getCurrentPosition() <= newTarget) {
                            rightFrontDrive.setPower(-rightSpeed);
                            rightRearDrive.setPower(-rightSpeed);
                        }
                        if(((DistanceSensor)colorSensor).getDistance(DistanceUnit.CM) <= WOBBLE_GOAL_DIST ){
                            leftServo.setPosition(CLOSED_LEFT_SERVO);
                            rightServo.setPosition(CLOSED_RIGHT_SERVO);
                        }
                        telemetry.addData("Left Encoder:", leftFrontDrive.getCurrentPosition());
                        telemetry.addData("Right Encoder:", -rightFrontDrive.getCurrentPosition());
                        telemetry.addData("Target Position:", newTarget);
                        telemetry.addLine("Target Direction: Forward");
                        telemetry.update();
                    }
                }
                // turn off motors
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftRearDrive.setPower(0);
                rightRearDrive.setPower(0);
                leftServo.setPosition(CLOSED_LEFT_SERVO);
                rightServo.setPosition(CLOSED_RIGHT_SERVO);
                sleep(500);
                wobbleLifter.setPower(-LIFT_POWER);
                sleep(800);
                wobbleLifter.setPower(0);

            }


        }

    /** Initialize the Vuforia localization engine */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**Initialize the TensorFlow Object Detection engine */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    /** Code used for IMU*/
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    private void rotate(int degrees, double speed) {
        double  leftSpeed;
        double rightSpeed;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftSpeed = -speed;
            rightSpeed = speed;
        }
        else if (degrees > 0)
        {   // turn left.
            leftSpeed = speed;
            rightSpeed = -speed;
        }
        else return;

        // set power to rotate.

        leftFrontDrive.setPower(leftSpeed);
        leftRearDrive.setPower(leftSpeed);
        rightFrontDrive.setPower(rightSpeed);
        rightRearDrive.setPower(rightSpeed);


        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);
        // wait for rotation to stop.
        sleep(200);

        // reset angle tracking on new heading.
        resetAngle();
    }


}
