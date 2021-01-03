package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.sql.Driver;
import java.util.List;

@Autonomous(name="Auto Red", group="Pushbot")
public class AutoRed extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            " AaDyemX/////AAABmTAtC2ONB0wHkUhRXShDQ5YMqzoOqj1zpW+eTxr9Vmb+zLP/5S8iHdsIsvHZDQDOgEQ8Q3j+Ke6cIggKMyK/1dGYnFe3e9oa/E/VCSqAwxd5EZZzKTEHnc+JATc5WBgX9zhdEFFZuuo1Xsn8Hpo93GCQC5n5q4uRhEAt7XvqRpj7qFT48aKhv2yHCraHMdrcD9NHXtr1CNpS53Qi9k/SrRvn9PdKL4taAey2C53xqvQ1j/+xh3Eh+3ORnMwaySnccNf145o9f5yv4fquBGYJfity4VIblSqAyg3VX/S/4aj8UmPe3T04Idl64z//OpS3ZXfozz/4Gk1qA9nW6twomt6e4kLrp3nLLiM6NOQGC1/e";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    /* Declare OpMode members. */
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftRearDrive;
    private DcMotor rightRearDrive;

    private ElapsedTime     runtime = new ElapsedTime();
    static final double COUNTS_PER_INCH = 531;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.4;
    static final double INTAKE_WHEEL_SPEED = 1;
    int rings = -1;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
        //Initialize the Motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftRearDrive  = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        // I believe that motors on the same side of the robot need to run in the same direction
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Ready To Start");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,

        //Drive to see rings - Encoder Drive

        //Detect Ring Stack - Detect Rings
        detectRings();
        /*
        // Power Shots  - Shoot 1 ring 3 times
        shootRings(1);
        //Line up next shot
        shootRings(1);
        //Line up next shot
        shootRings(1);
       //Determine where to place wobble goal and next autonomous steps
         switch(rings){
             case 0:
                 break;
             case 1:
                 break;
             case 4:
                 break;
         }

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
        //encoderDrive(TURN_SPEED,   12,  4.0);  //Forward 12 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    
        public void encoderDrive(double speed, double inches, double timeoutS) {
        int newTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newTarget = leftFrontDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                //reset runtime
                runtime.reset();

                if (leftFrontDrive.getCurrentPosition() > newTarget) { //If current position is larger than new target, robot needs to reverse

                    while (leftFrontDrive.getCurrentPosition() >= newTarget && (runtime.seconds() < timeoutS) && opModeIsActive()) {
                        leftFrontDrive.setPower(-speed);
                        rightFrontDrive.setPower(-speed);
                        leftRearDrive.setPower(-speed);
                        rightRearDrive.setPower(-speed);
                    }
                } else { //If current position is smaller than new target, robot goes forward
                    while (leftFrontDrive.getCurrentPosition() <= newTarget && (runtime.seconds() < timeoutS) && opModeIsActive()) {
                        leftFrontDrive.setPower(speed);
                        rightFrontDrive.setPower(speed);
                        leftRearDrive.setPower(speed);
                        rightRearDrive.setPower(speed);
                    }
                }

                // Stop all motion;
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftRearDrive.setPower(0);
                rightRearDrive.setPower(0);

                //  sleep(250);   // optional pause after each move
            }
        }

        public void intakeWheelDrive(double speed, double inches, double timeoutS){
            int newTarget;
            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newTarget = leftFrontDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                //reset runtime
                runtime.reset();

                if (leftFrontDrive.getCurrentPosition() > newTarget) { //If current position is larger than new target, robot needs to reverse

                    while (leftFrontDrive.getCurrentPosition() >= newTarget && (runtime.seconds() < timeoutS) && opModeIsActive()) {
                        leftFrontDrive.setPower(-speed);
                        rightFrontDrive.setPower(-speed);
                        leftRearDrive.setPower(-speed);
                        rightRearDrive.setPower(-speed);
                        //INTAKE WHEELS SPIN
                    }
                } else { //If current position is smaller than new target, robot goes forward
                    while (leftFrontDrive.getCurrentPosition() <= newTarget && (runtime.seconds() < timeoutS) && opModeIsActive()) {
                        leftFrontDrive.setPower(speed);
                        rightFrontDrive.setPower(speed);
                        leftRearDrive.setPower(speed);
                        rightRearDrive.setPower(speed);
                        //INTAKE WHEELS SPIN
                    }
                }

                // Stop all motion;
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftRearDrive.setPower(0);
                rightRearDrive.setPower(0);
                //INTAKE WHEELS STOP

                //  sleep(250);   // optional pause after each move
            }
        }

        public void detectRings(){
            //Webcam Detection changes rings to number of rings- defaults to 0
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

        public void shootRings(int amountToShoot){
            //Launch rings for specific amount of time, according to amount to shoot
        }

        public void wobbleClawOpen(){
            //Set wobble claw position to be open
        }
    /**
     * Initialize the Vuforia localization engine.
     */
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

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


    }
