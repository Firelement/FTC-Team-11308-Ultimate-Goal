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


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Driver Control", group="Linear Opmode")
public class DriverControl extends LinearOpMode {

    //Declare constants
    //If you have a value that stays constant is is good practice to make it a constant and
    //define it at the beginning of the program where it can easily accessed.
    private final double LIFT_POWER = 0.8;
    private final double CLOSED_LEFT_SERVO = 0.0;
    private final double CLOSED_RIGHT_SERVO = 0.6;
    private final double OPEN_LEFT_SERVO = 0.6;
    private final double OPEN_RIGHT_SERVO = 0.0;
    private final double WOBBLE_GOAL_DIST = 7.0;
    private final double FLYWHEEL_POWER = 0.6;//This value may need additional logic if we need to vary the power.
    private final double FLYWHEEL_POWERSHOT = .55;
    private final double INTAKE_POWER1 = 0.75;// This value has not been tested.
    private final double INTAKE_POWER2 = 0.5;
    private final double RING_STOPPER_POWER = -1.0;// This value has not been tested yet either;
    private final double SLOW_MODE_CONSTANT = 0.4;//This value is used to scale down the motor power

    //We may need a servo to release the intake.
    private final double INTAKE_RELEASE_POWER = 1.0;

    // Declare Motor Classes
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftRearDrive;
    private DcMotor rightRearDrive;
    private DcMotor wobbleLifter;
    private DcMotor intake1;
    private DcMotor intake2;
    private DcMotor flyWheel;

    //Declare Servo Classes
    private CRServo ringStopper;
    private CRServo intakeRelease;
    private Servo rightServo;
    private Servo leftServo;

    // Color sensor
    NormalizedColorSensor colorSensor;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Here we initialize the Motor,Servo, and Sensors to the hardware configured on the robot.

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
        ringStopper = hardwareMap.get(CRServo.class,"ring_stopper");
        intakeRelease = hardwareMap.get(CRServo.class,"intake_release");
        leftServo = hardwareMap.get(Servo.class,"leftServo");
        rightServo = hardwareMap.get(Servo.class,"rightServo");

        //Initialize Color sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // Most robots need the motors on one side to be reversed to drive properly
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        wobbleLifter.setDirection(DcMotor.Direction.FORWARD);
        intake1.setDirection(DcMotor.Direction.REVERSE);
        intake2.setDirection(DcMotor.Direction.FORWARD);
        flyWheel.setDirection(DcMotor.Direction.REVERSE);
        flyWheel.setPower(0.0);

        //Initialize the Servo positions
        leftServo.setPosition(CLOSED_LEFT_SERVO);
        rightServo.setPosition(CLOSED_RIGHT_SERVO);
        ringStopper.setPower(RING_STOPPER_POWER);
        intakeRelease.setPower(INTAKE_RELEASE_POWER);

        //This Boolean is part of the odd logic that Andrew wants for the wobble goal lifter.
        // Mode 1 is where the hand toggles between open and closed when <> or <> buttons are pressed
        // Mode 2 is after the button on the hand has been pressed. The hand will default to
        // closed but will open while the <> button is held.   <> means button not decided.
        boolean isInMode2 = false;
        //This variable allows the servo to toggle position while in mode 1 it must be defined here
        // so that the hand will hold its position after the button is released.
        double leftServoPos = CLOSED_LEFT_SERVO;
        double rightServoPos = CLOSED_RIGHT_SERVO;
        double intakeReleasePower = INTAKE_RELEASE_POWER;

        //These variables are necessary for slow mode to work.
        boolean isInSlowMode = false;
        boolean oldYbuttonState = false;

        //Fly wheel power
        double flyWheelPower = 0.0;

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
            double intakePower1;
            double intakePower2;

            // Setup variables to hold the Game pad inputs
            double drive;
            double rotate;
            // positive rotation is in the clockwise direction
            double strafe;

            //Setup variables to hold the Servo Positions
            //double intakeReleasePosition;
            //double ringStopperPosition;


            // Assign the Game pad inputs
            // These can be changed to what ever the driver wants it to be
            //some of these may need to be made negative depending  on the which direction
            //is positive on the gamepad
            drive = gamepad1.left_stick_y;
            rotate = -gamepad1.right_stick_x;
            strafe = -gamepad1.left_stick_x;

            // Assign the power variables to the Game pad inputs
            leftFrontPower = drive + rotate + strafe;
            leftRearPower = drive + rotate - strafe;
            rightFrontPower = drive - rotate - strafe;
            rightRearPower = drive - rotate + strafe;

            //allow the user to toggle between slow and fast modes
            if(gamepad1.y == true && oldYbuttonState == false){
                isInSlowMode = !isInSlowMode;
            }
            oldYbuttonState = gamepad1.y;
            //if we are in slow mode for grabbing the wobble goal
            if(isInSlowMode == true){
                leftFrontPower *= SLOW_MODE_CONSTANT;
                leftRearPower *= SLOW_MODE_CONSTANT;
                rightFrontPower *= SLOW_MODE_CONSTANT;
                rightRearPower *= SLOW_MODE_CONSTANT;
            }

            //Set the wobble lifter power
            //Alternate but less clear code: if(gamepad1.left_bumper){
            if(gamepad2.right_bumper == true){
                wobbleLifterPower = LIFT_POWER;
            }
            else if(gamepad2.left_bumper == true){
                wobbleLifterPower = -LIFT_POWER;
            }
            else{
                wobbleLifterPower = 0.0;
            }

            //set the Intake power
            // we have to set it above the code for the fly wheel(line 238-251) as that code may
            //need to override this intake value to load the rings into the launcher.
            if(gamepad1.right_bumper == true){
                intakePower1 = INTAKE_POWER1;
                intakePower2 = INTAKE_POWER2;
            }
            else if (gamepad1.left_bumper == true){
                intakePower1 = -INTAKE_POWER1;
                intakePower2 = -INTAKE_POWER2;
            }
            else if (gamepad2.y  == true){
                intakePower1 = INTAKE_POWER1;
                intakePower2 = INTAKE_POWER2;
            }
            else{
                intakePower1 = 0.0;
                intakePower2 = 0.0;
            }

            // make sure that the power variables are within -1.0 and 1.0
            leftFrontPower    = Range.clip(leftFrontPower, -1.0, 1.0) ;
            rightFrontPower   = Range.clip(rightFrontPower, -1.0, 1.0) ;
            leftRearPower     = Range.clip(leftRearPower,-1.0,1.0);
            rightRearPower    = Range.clip(rightRearPower,-1.0,1.0);

            //set the Servo positions based on the driver control input
            //if we are in mode 1 use A andB buttons to toggle open and close
            if(isInMode2 == false){
                if(gamepad2.a == true){
                    leftServoPos = OPEN_LEFT_SERVO;
                    rightServoPos = OPEN_RIGHT_SERVO;
                }
                else if(gamepad2.b == true){
                    leftServoPos = CLOSED_LEFT_SERVO;
                    rightServoPos = CLOSED_RIGHT_SERVO;
                }
            }
            //If we are in mode 2 set the default position to closed. Allow the user to open the hand
            //by holding the A button.
            else {
                if (gamepad2.a == true) {
                    leftServoPos = OPEN_LEFT_SERVO;
                    rightServoPos = OPEN_RIGHT_SERVO;
                }
                else {
                    leftServoPos = CLOSED_LEFT_SERVO;
                    rightServoPos = CLOSED_RIGHT_SERVO;
                }
            }
            //If the color sensor senses the wobble goal and the hand is open, go to mode 2
            if(((DistanceSensor)colorSensor).getDistance(DistanceUnit.CM) <= WOBBLE_GOAL_DIST && rightServoPos == OPEN_RIGHT_SERVO){
                isInMode2 = true;
            }
            //Allow the user to reset to Mode 1 when the X button is pressed
            if(gamepad2.x == true){
                isInMode2 = false;
            }

            //Set the position of the servos
            leftServo.setPosition(leftServoPos);
            rightServo.setPosition(rightServoPos);

            //This section controls launching the ring
            // Release the ring
            if (gamepad2.right_trigger > 0.2) {
                ringStopper.setPower(RING_STOPPER_POWER);
            }
            else if (gamepad2.left_trigger > 0.2){
                ringStopper.setPower(-RING_STOPPER_POWER);
            }
            else {
                ringStopper.setPower(0.0);
            }

            //this section will be commented out until we have an intake release servo
            if(gamepad1.a == true){
                intakeReleasePower = INTAKE_RELEASE_POWER;
            }
            else{
                intakeReleasePower = 0.0;
            }

            //toggle fly wheel power
            if(gamepad2.dpad_up == true){
                flyWheelPower = FLYWHEEL_POWER;
            }
            else if(gamepad2.dpad_down){
                flyWheelPower = FLYWHEEL_POWERSHOT;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftRearDrive.setPower(leftRearPower);
            rightRearDrive.setPower(rightRearPower);

            //set the power of the wobble lifter
            wobbleLifter.setPower(wobbleLifterPower);

            //set the power of the fly wheel
            flyWheel.setPower(flyWheelPower);

            //set the power of the intake
            intake1.setPower(intakePower1);
            intake2.setPower(intakePower2);

            //Set the position of the servos
            leftServo.setPosition(leftServoPos);
            rightServo.setPosition(rightServoPos);
            intakeRelease.setPower(intakeReleasePower);

            // Show the elapsed game time, wheel power and color sensor distance.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Front Motor Power:",leftFrontPower);
            telemetry.addData("Right Front Motor Power:",rightFrontPower);
            telemetry.addData("Left Rear Motor Power:",leftRearPower);
            telemetry.addData("Right Rear Motor Power:",rightRearPower);
            telemetry.addData("Distance (cm)", "%.3f",
                    ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
