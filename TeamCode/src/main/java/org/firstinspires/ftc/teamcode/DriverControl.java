/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//This program is disabled because it is not necessary for testing purposes right now.
//This program is where the final driver control program will be implemented.
@Disabled
@TeleOp(name="Driver Control", group="Linear Opmode")
public class DriverControl extends LinearOpMode {

    //Declare constants
    //If you have a value that stays constant is is good practice to make it a constant and
    //define it at the beginning of the program where it can easily accessed.
    private final double LIFT_POWER = 0.6;
    private final double CLOSED_HAND_POSITION = 0.0;
    private final double OPEN_HAND_POSITION = 0.5;
    private final double FLYWHEEL_POWER = 0.8;
    private final double INTAKE_POWER = 0.5;
    private final double CLOSED_RING_STOPPER = 0.5;// This value has not been tested yet.
    private final double OPEN_RING_STOPPER = 0.0;// This value has not been tested yet either;

    //We may need a servo to release the intake.
    //private final double INTAKE_RELEASE_LATCHED_POSITION = 0.3;
    //private final double INTAKE_RELEASE_OPEN_POSITION = 0.7;

    // Declare Motor Classes
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftRearDrive;
    private DcMotor rightRearDrive;
    private DcMotor wobbleLifter;
    private DcMotor intake;
    private DcMotor flyWheel;

    //Declare Servo Classes
    private Servo wobbleHand;
    private Servo ringStopper;
    //this servo is not currently part of the design
    //private Servo intakeRelease;

    //Declare Buttons
    private DigitalChannel wobbleButton;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Here we initialize the Motor,Servo, and Sensors to the hardware configured on the robot.

        //Initialize the Motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftRearDrive  = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");
        wobbleLifter = hardwareMap.get(DcMotor.class,"wobble_Lifter");
        intake = hardwareMap.get(DcMotor.class,"intake");
        flyWheel = hardwareMap.get(DcMotor.class,"fly_wheel");

        //Initialize Servos
        wobbleHand = hardwareMap.get(Servo.class,"wobble_hand");
        ringStopper = hardwareMap.get(Servo.class,"ring_stopper");
        //intakeRelease = hardwareMap.get(Servo.class,"intake_release");

        //Initialize buttons
        wobbleButton = hardwareMap.get(DigitalChannel.class,"wobble_button");

        // Most robots need the motors on one side to be reversed to drive properly
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        wobbleLifter.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        flyWheel.setDirection(DcMotor.Direction.FORWARD);

        //Initialize the Servo positions
        wobbleHand.setPosition(CLOSED_HAND_POSITION);
        ringStopper.setPosition(CLOSED_RING_STOPPER);
        //intakeRelease.setPosition(INTAKE_RELEASE_LATCHED_POSITION);

        //Specify that motorOnButton is an input
        wobbleButton.setMode(DigitalChannel.Mode.INPUT);

        //Set up a variable to hold the wobble hand position. It must be defined
        //here so that the hand will hold its position after the gamepad button is released
        double wobbleHandPosition = CLOSED_HAND_POSITION;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to hold power level
            double leftFrontPower;
            double rightFrontPower;
            double leftRearPower;
            double rightRearPower;

            //Setup a variable for the wobbleLifter and flywheel
            double wobbleLifterPower;
            double flywheelPower;
            double intakePower;

            // Setup variables to hold the Game pad inputs
            double drive;
            double rotate;
            // positive rotation is in the clockwise direction
            double strafe;

            //Setup variables to hold the Servo Positions
            //double intakeReleasePosition;
            double ringStopperPosition;


            // Assign the Game pad inputs
            // These can be changed to what ever the driver wants it to be
            //some of these may need to be made negative depending  on the which direction
            //is positive on the gamepad
            drive = gamepad1.right_stick_y;
            rotate = -gamepad1.left_stick_x;
            strafe = -gamepad1.right_stick_x;

            // Assign the power variables to the Game pad inputs
            leftFrontPower = drive + rotate + strafe;
            leftRearPower = drive + rotate - strafe;
            rightFrontPower = drive - rotate - strafe;
            rightRearPower = drive - rotate + strafe;

            //Set the wobble lifter power
            //Alternate but less clear code: if(gamepad1.left_bumper){
            if(gamepad1.left_bumper == true){
                wobbleLifterPower = LIFT_POWER;
            }
            else if(gamepad1.right_bumper == true){
                wobbleLifterPower = -LIFT_POWER;
            }
            else{
                wobbleLifterPower = 0.0;
            }

            // make sure that the power variables are within -1.0 and 1.0
            leftFrontPower    = Range.clip(leftFrontPower, -1.0, 1.0) ;
            rightFrontPower   = Range.clip(rightFrontPower, -1.0, 1.0) ;
            leftRearPower     = Range.clip(leftRearPower,-1.0,1.0);
            rightRearPower    = Range.clip(rightRearPower,-1.0,1.0);

            //set the Servo positions based on the driver control input
            if(gamepad1.a == true){
                wobbleHandPosition = OPEN_HAND_POSITION;
            }
            else if(gamepad1.b == true){
                wobbleHandPosition = CLOSED_HAND_POSITION;
            }
            //This section will be commented out unless we decide we are going to use a button
            //in the wobble goal hand
            /*
            else{
                if(wobbleButton.getState() == true){
                    wobbleHandPosition = CLOSED_HAND_POSITION;
                }
                else{
                    wobbleHandPosition = OPEN_HAND_POSITION;
                }
            }
            */

            //This section controls launching the ring
            if(gamepad1.right_trigger > 0.1) {  //I am using a deadband here because the trigger returns a double
                //Spin up the flywheel
                flywheelPower = FLYWHEEL_POWER;
            }
            else if (gamepad1.right_bumper == true) {
                //keep the fly wheel spinning
                flywheelPower = FLYWHEEL_POWER;
                ringStopperPosition = OPEN_RING_STOPPER;
                //intakePower = INTAKE_POWER;
            }
            else {
                flywheelPower = 0;
                ringStopperPosition = CLOSED_RING_STOPPER;
            }

            //this section will be commented out untill we have an intake
            /*
            if(gamepad1.x == true){
                intakeReleasePosition = INTAKE_RELEASE_OPEN_POSITION;
            }
            else{
                intakeReleasePosition = INTAKE_RELEASE_LATCHED_POSITION;
            }
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftRearDrive.setPower(leftRearPower);
            rightRearDrive.setPower(rightRearPower);

            //set the power of the wobble lifter
            wobbleLifter.setPower(wobbleLifterPower);

            //Set the position of the servos
            wobbleHand.setPosition(wobbleHandPosition);
            //intakeRelease.setPosition(intakeReleasePosition);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Front Motor Power:",leftFrontPower);
            telemetry.addData("Right Front Motor Power:",rightFrontPower);
            telemetry.addData("Left Rear Motor Power:",leftRearPower);
            telemetry.addData("Right Rear Motor Power:",rightRearPower);
            telemetry.update();
        }
    }
}
