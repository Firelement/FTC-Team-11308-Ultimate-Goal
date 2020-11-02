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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Set;


@TeleOp(name="Simple Drive Program", group="Linear Opmode")
public class Driver_Control_Test extends LinearOpMode {
    
    //Declare some variables to hold the driver input and motor power
    double left;
    double right;
    double drive;
    double turn;
    double launcherpower;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    //Declare Motors
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor launcher;
    
    // Declare Button
    private DigitalChannel motorOnButton;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class,"rightDrive");
        launcher = hardwareMap.get(DcMotor.class,"ringLauncher");
        motorOnButton = hardwareMap.get(DigitalChannel.class, "motorOnButton");
        
        // Set the direction of the motors
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        //Specify that motorOnButton is an input
        motorOnButton.setMode(DigitalChannel.Mode.INPUT);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){

            // read driver input
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;
            
            // if the A button is pressed run the launcher forwards
            // if the Y button is pressed run it backwards
            if(gamepad1.a == true){
                launcherpower = 1.0;
            }
            else if(gamepad1.y == true){
                launcherpower = -1.0;
            }
            else{
                launcherpower = 0.0;
            }

            // Combine drive and turn for blended motion and make sure the value is safe
            left    = Range.clip(drive + turn, -1.0, 1.0) ;
            right   = Range.clip(drive - turn, -1.0, 1.0) ;
            launcherpower = Range.clip(launcherpower,-1.0,1.0);
            
            //set the motor power
            rightDrive.setPower(right);
            leftDrive.setPower(left);
            launcher.setPower(launcherpower);
            
            // debugging telemetry.
            telemetry.addLine( "Touch Sensor Value: "+ motorOnButton.getState());
            telemetry.addLine("Left Motor Encoder Value: "+ leftDrive.getCurrentPosition() );
            telemetry.addLine("Right Motor Encoder Value: "+ rightDrive.getCurrentPosition() );
            telemetry.update();
        }
    }
}
