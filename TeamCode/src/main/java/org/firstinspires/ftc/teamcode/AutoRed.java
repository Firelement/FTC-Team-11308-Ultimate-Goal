package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Driver;

@Autonomous(name="Auto Red", group="Pushbot")
public class AutoRed extends LinearOpMode {

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
    int rings = 0;

    @Override
    public void runOpMode() {

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


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,

        //Drive to see rings - Encoder Drive

        //Detect Ring Stack - Detect Rings
        detectRings();
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
        }

        public void shootRings(int amountToShoot){
            //Launch rings for specific amount of time, according to amount to shoot
        }

        public void wobbleClawOpen(){
            //Set wobble claw position to be open
        }
    }
