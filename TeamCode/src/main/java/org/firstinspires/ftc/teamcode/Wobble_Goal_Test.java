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


@TeleOp(name="Wobble Goal Test", group="Linear Opmode")
public class Wobble_Goal_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor liftMotor;
    private Servo latchServo;
    private DigitalChannel button;

    //constants
    private final double LIFT_POWER = 0.6;
    private final double CLOSED_SERVO_POSITION = 0.28;
    private final double OPEN_SERVO_POSITION = 0.7;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        liftMotor = hardwareMap.get(DcMotor.class, "wobble_lifter");
        latchServo = hardwareMap.get(Servo.class,"latchservo");
        button = hardwareMap.get(DigitalChannel.class,"button");

        // Most robots need the motor on one side to be reversed to drive forward
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        //Specify that motorOnButton is an input
        button.setMode(DigitalChannel.Mode.INPUT);

        //variable for the servo position
        double servoPosition = OPEN_SERVO_POSITION;

        boolean wasButtonActivated = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for the motor power
            double motorPower;

            //set the motor power based on the user input
            if(gamepad1.left_bumper == true){
                motorPower = LIFT_POWER;
            }
            else if(gamepad1.right_bumper == true){
                motorPower = -LIFT_POWER;
            }
            else{
                motorPower = 0.0;
            }

            //safety check on the motor power
            motorPower    = Range.clip(motorPower, -1.0, 1.0) ;

            //set the servo position based on the user input
            if(gamepad1.a == true){
                servoPosition = OPEN_SERVO_POSITION;
            }
            else if(gamepad1.b == true){
                servoPosition = CLOSED_SERVO_POSITION;
            }
            else{
                if(wasButtonActivated) {
                    servoPosition = CLOSED_SERVO_POSITION;
                }
            }

            //if the button is pressed set wasButtonActivated to true to lock the hand closed
            if(button.getState() == false){
                wasButtonActivated = true;
            }
            //allow the user to reset wasButtonActivated
            if(gamepad1.x == true){
                wasButtonActivated = false;
            }

            // Send the power to the motor
            liftMotor.setPower(motorPower);

            //set the servo position
            latchServo.setPosition(servoPosition);
            
            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine("Button state:"+button.getState());
            telemetry.update();
        }
    }
}
