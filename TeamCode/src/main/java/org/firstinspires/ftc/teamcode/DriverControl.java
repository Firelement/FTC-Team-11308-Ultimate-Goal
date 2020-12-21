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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//This program is disabled because it is not necessary for testing purposes right now.
//This program is where the final driver control program will be implemented.
@Disabled
@TeleOp(name="Driver Control", group="Linear Opmode")
public class DriverControl extends LinearOpMode {

    //Declare constants
    private final double LIFT_POWER = 0.6;
    private final double CLOSED_LATCH_POSITION = 0.0;
    private final double OPEN_LATCH_POSITION = 0.5;
    //private final double INTAKE_RELEASE_LATCHED_POSITIION = 0.3;
    //private final double INTAKE_RELEASE_OPEN_POSIION = 0.7;

    // Declare Motor Classes
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftRearDrive;
    private DcMotor rightRearDrive;
    private DcMotor wobbleLifter;
    private DcMotor intake;
    private DcMotor flyWheel;
    //This is the alternate code if we go for Austin's launcher
    //private DcMotor boltRetractMotor;

    //Declare Servo Classes
    private Servo wobbleLiftLatch;
    //this servo is not currently part of the design
    //private Servo intakeRelease;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Initialize the Motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftRearDrive  = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");
        wobbleLifter = hardwareMap.get(DcMotor.class,"wobble_Lifter");
        intake = hardwareMap.get(DcMotor.class,"intake");
        flyWheel = hardwareMap.get(DcMotor.class,"fly_wheel");
        //boltRetractMotor =hardwareMap.get(DcMotor.class,"bolt_retract_motor");

        //Initialize Servos
        wobbleLiftLatch = hardwareMap.get(Servo.class,"wobble_lift_latch");
        //intakeRelease = hardwareMap.get(Servo.class,"intake_release");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        // I believe that motors on the same side of the robot need to run in the same direction
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        wobbleLifter.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        flyWheel.setDirection(DcMotor.Direction.FORWARD);
        //boltRetractMotor.setDirection(DcMotor.Direction.FORWARD);

        //Intitialize the Servo positions
        wobbleLiftLatch.setPosition(CLOSED_LATCH_POSITION);
        //intakeRelease.setPosition(INTAKE_RELEASE_LATCHED_POSITIION);


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

            //Setup a variable for the wobbleLifter
            double wobbleLifterPower;

            // Setup variables to hold the Game pad inputs
            double drive;
            double rotate;
            // positive rotation is in the clockwise direction
            double strafe;

            //Setup variables to hold the Servo Positions
            double intakeReleasePosition;
            double wobbleLiftLatchPosition;


            // Assign the Game pad inputs
            // These can be changed to what ever the driver wants it to be
            //some of these may need to be made negative depending  on the gamepad
            drive = gamepad1.right_stick_y;
            rotate = -gamepad1.left_stick_x;
            strafe = -gamepad1.right_stick_x;

            // Assign the power variables to the Game pad inputs
            // currently strafing is not implemented
            leftFrontPower = drive + rotate + strafe;
            leftRearPower = drive + rotate - strafe;
            rightFrontPower = drive - rotate - strafe;
            rightRearPower = drive - rotate + strafe;

            //Set the wobble lifter power
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
            if(gamepad1.x == true){
                wobbleLiftLatchPosition = OPEN_LATCH_POSITION;
            }
            else{
                wobbleLiftLatchPosition = CLOSED_LATCH_POSITION;
            }
            //this section will be commented out untill we have an intake
            /*
            if(gamepad1.a == true){
                intakeReleasePosition = INTAKE_RELEASE_OPEN_POSIION;
            }
            else{
                intakeReleasePosition = INTAKE_RELEASE_LATCHED_POSITIION;
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
            wobbleLiftLatch.setPosition(wobbleLiftLatchPosition);
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
