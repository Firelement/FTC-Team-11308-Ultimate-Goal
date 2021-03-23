package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
    private final double FLYWHEEL_POWER = 0.6;//This value may need additional logic if we need to vary the power.
    private final double FLYWHEEL_POWERSHOT = 0.5;

    //Intake
    private final double INTAKE_POWER1 = 0.75;// This value has not been tested.
    private final double INTAKE_POWER2 = 0.5;
    private final double RING_STOPPER_POWER = -1.0;// This value has not been tested yet either;
    private final double INTAKE_RELEASE_POWER = 1.0;


    //Timing
    private ElapsedTime runtime = new ElapsedTime();

    //Drive
    private static final double COUNTS_PER_INCH = 1400; //Counts on odometry wheel per inch of distance
    private static final double DRIVE_SPEED = 0.5; //Speed of wheel
    private static final double STRAFE_SPEED = 0.3;

    private static double DRIVE_ADJUSTMENT = 0; //Variable for power adjustment
    private static final double POWER_ADJUSTMENT_CONSTANT = 10000.0;    //Variable that dictates how much power adjustment will be added based on encoder difference in odometry

    //Other variables
    private static int rings = 0;    //Ring count variable for pathing
    private static int heldRings = 3; // Amount of held rings in robot

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
    //private Servo ringStopper;
    //this servo is not currently part of the design
    //private Servo intakeRelease;
    private CRServo ringStopper;
    private CRServo intakeRelease;
    private Servo rightServo;
    private Servo leftServo;

    // Color sensor
    NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() {

        // Activate TensorFlow Object Detection before we wait for the start command
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }

        //Initialize the Motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftRearDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        wobbleLifter = hardwareMap.get(DcMotor.class,"wobble_lifter");
        intake1 = hardwareMap.get(DcMotor.class,"intake1");
        intake2 = hardwareMap.get(DcMotor.class,"intake2");
        flyWheel = hardwareMap.get(DcMotor.class,"fly_wheel");

        //Initialize Servos
        //ringStopper = hardwareMap.get(Servo.class,"ring_stopper");
        intakeRelease = hardwareMap.get(CRServo.class,"intake_release");
        ringStopper = hardwareMap.get(CRServo.class,"ring_stopper");
        intakeRelease = hardwareMap.get(CRServo.class,"intake_release");
        leftServo = hardwareMap.get(Servo.class,"leftServo");
        rightServo = hardwareMap.get(Servo.class,"rightServo");

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

        telemetry.addLine("Ready To Start");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /* Autonomous section */

        //Turn on flywheel to allow spinup time
        flyWheel.setPower(FLYWHEEL_POWERSHOT);
        //Drive to see rings - Encoder Drive
        encoderDriveY(DRIVE_SPEED, 30, 5);
        //Detect Ring Stack - Detect Rings
        detectRings();
        //Drive to power shot shooting location
        encoderDriveX(STRAFE_SPEED, -55, 5);
        //Release intake servo during movement
        intakeRelease.setPower(INTAKE_RELEASE_POWER);
        encoderDriveY(DRIVE_SPEED, 31, 5);
        intakeRelease.setPower(0);
        //Power Shots  - Shoot 1 ring 3 times
        //Raise fire blocker
        ringStopper.setPower(RING_STOPPER_POWER);
        sleep(500);
        ringStopper.setPower(0);
        //Shoot ring 1
        shootRings(1);
        //Line up next shot
        encoderDriveX(STRAFE_SPEED, 7.5, 2);
        //Shoot ring 2
        shootRings(1);
        //Line up next shot
        encoderDriveX(STRAFE_SPEED, 7.5, 2);
        //Shoot final ring
        shootRings(1);
        //Determine where to place wobble goal and next autonomous steps

        if(rings == 0) {//0 Rings
            //Drive to wobble goal deposit area
            encoderDriveX(STRAFE_SPEED, 55, 5);
            encoderDriveY(DRIVE_SPEED, -10, 1);
            //Drop wobble goal
            dropWobbleGoal();
            //Park on line
            encoderDriveY(DRIVE_SPEED, 3, 1);
        }else if(rings == 1) {//1 Ring
            //Drive to wobble goal deposit area
            encoderDriveX(STRAFE_SPEED, 26, 3);
            encoderDriveY(DRIVE_SPEED, 25, 3);
            //Drop wobble goal
            dropWobbleGoal();
            //Grab last ring for shooting
            encoderDriveY(DRIVE_SPEED, -25, 3);
            encoderDriveX(STRAFE_SPEED, -6, 1);
            intakeWheelDrive(DRIVE_SPEED, -35, 6);
            heldRings = 1;
            //Drive to goal shooting location
            encoderDriveY(DRIVE_SPEED, 35, 6);
            //Shoot ring
            shootRings(1);
            //Park on line
            encoderDriveY(DRIVE_SPEED, 5, 1);
        }else { //4 Rings
            //Drive to wobble goal deposit area
            encoderDriveX(STRAFE_SPEED, 55, 5);
            encoderDriveY(DRIVE_SPEED, 48, 5);
            //Drop wobble goal
            dropWobbleGoal();
            //Grab last ring for shooting
            encoderDriveY(DRIVE_SPEED, -48, 3);
            encoderDriveX(STRAFE_SPEED, -28, 1);
            intakeWheelDrive(DRIVE_SPEED, -35, 6);
            heldRings = 3;
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
            encoderDriveY(DRIVE_SPEED, 5, 1);
        }
        //Turn flywheel off
        flyWheel.setPower(0);


        /*If: 4 rings
        Deposit wobble goal far - Open Wobble Claw
        Grab Rings - Intake Wheel Drive
        Shoot high goal - Shoot Rings
        Grab Last ring - Intake Wheel Drive
        Shoot high goal - Shoot Rings
        Park on line - Encoder Drive
        */

        /*If: 1 ring
        Deposit wobble goal middle - Open Wobble Claw
        Grab Ring - Intake Wheel Drive
        Shoot high goal - Shoot Rings
        Park on line - Encoder Drive
        */

        /*If: No Rings
        Deposit wobble goal closest - Open Wobble Claw
        Park on line - Encoder Drive
        */
              telemetry.addData("Path", "Complete");
              telemetry.update();

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
                        leftSpeed = speed - DRIVE_ADJUSTMENT;
                        rightSpeed = speed + DRIVE_ADJUSTMENT;

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
                  sleep(250);   // optional pause after each move
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

                while (rightRearDrive.getCurrentPosition() >= newTarget && (runtime.seconds() < timeoutS) && opModeIsActive()) {
                    leftFrontDrive.setPower(speed);
                    rightFrontDrive.setPower(-speed);
                    leftRearDrive.setPower(-speed);
                    rightRearDrive.setPower(speed);
                }
            } else { //If current position is smaller than new target, robot goes left
                while (rightRearDrive.getCurrentPosition() <= newTarget && (runtime.seconds() < timeoutS) && opModeIsActive()) {
                    leftFrontDrive.setPower(-speed);
                    rightFrontDrive.setPower(speed);
                    leftRearDrive.setPower(speed);
                    rightRearDrive.setPower(-speed);
                }
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftRearDrive.setPower(0);
            rightRearDrive.setPower(0);
            sleep(250);   // optional pause after each move
        }
    }

        /** Driving  with intake on */
        public void intakeWheelDrive(double speed, double inches, double timeoutS){
            //Intake wheels on
            intake1.setPower(INTAKE_POWER1);
            intake2.setPower(INTAKE_POWER2);
            //Drive specified distance
            encoderDriveY(speed,inches,timeoutS);
            //Possible pause to allow rings to be held
            sleep(250);
            //Intake wheels off
            intake1.setPower(0);
            intake2.setPower(0);
        }

        /** Detect the amount of rings*/
        public void detectRings(){
            //Webcam Detection changes rings to number of rings- defaults to 0
           sleep(500);
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null && rings == -1) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        if(recognition.getLabel().equalsIgnoreCase( "quad")){
                            rings = 4;
                        }else if(recognition.getLabel().equalsIgnoreCase("single")){
                            rings = 1;
                        }else{
                            rings = 0;
                        }
                    }
                    telemetry.update();
                }
            }
        }

        /** Shoot rings*/
        public void shootRings(int amountToShoot){
            //Launch rings for specific amount of time, according to amount to shoot
            if(amountToShoot == 1){
                flyWheel.setPower(FLYWHEEL_POWERSHOT);
            }else {
                flyWheel.setPower(FLYWHEEL_POWER);
            }
            sleep(500);
            intake1.setPower(INTAKE_POWER1);
            intake2.setPower(INTAKE_POWER2);
            for(int i = amountToShoot; i >0; i--){
                if(heldRings>1) {// if not last ring
                    //Shoot ring by continuing to run intake
                    sleep(800);
                    heldRings--;
                }else{// if last ring
                    //Rotate fire blocker to launch last ring
                    ringStopper.setPower(RING_STOPPER_POWER);
                    sleep(1500);
                    heldRings--;
                }
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
            sleep(1000);
            wobbleLifter.setPower(0);
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

    }
