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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="WheelLauncher Test Program", group="Linear Opmode")
public class WheelLauncherTestProgram extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor flywheel = null;

    private int lastencoder = 0;
    private double lastchecktime;
    private double currentPower = 0.0;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     POWER_INCREMENT = 0.1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.

        flywheel  = hardwareMap.get(DcMotor.class, "flywheel");
        // Wait for the game to start (driver presses PLAY)

        //enable the encoder and save the value
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastencoder = flywheel.getCurrentPosition();


        waitForStart();
        runtime.reset();

        //set the saved time
        lastchecktime = runtime.milliseconds();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){

            //allow the user to increment or deincrement the power of the motor
            if(gamepad1.right_bumper&&currentPower<= 1.0 - POWER_INCREMENT){
                currentPower +=POWER_INCREMENT;
            }
            else if(gamepad1.left_bumper&&currentPower >= -1.0 + POWER_INCREMENT){
                currentPower -=POWER_INCREMENT;
            }
            //Saftey Check
            currentPower = Range.clip(currentPower,-1.0,1.0);
            flywheel.setPower(currentPower);

            //calculate the rpms
            double rpms = ((flywheel.getCurrentPosition()-lastencoder)/COUNTS_PER_MOTOR_REV)/(runtime.milliseconds() -lastchecktime);

            //save the current time and encoder value;
            lastencoder = flywheel.getCurrentPosition();
            lastchecktime = runtime.milliseconds();

            //send the power and rpms to the driver station
            telemetry.addLine("Motor Power: "+ currentPower );
            telemetry.addLine("Rpms :"+ rpms);
            telemetry.update();

        }
    }
}
