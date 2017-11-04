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
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;


import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


@TeleOp(name="Auton", group="Auton")

public class BracketSourcesSAUTO extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareBracketBot robot           = new HardwareBracketBot();   // Use a Pushbot's hardware
    ColorSensor sensorColor;
    // could also use HardwarePushbotMatrix class.

    @Override
    public void runOpMode() {


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        color_sensor = hardwareMap.colorSensor.get("color");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        Jewel.setPosition(0.9)

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            //maxes the values at 1
           FLvalue = Range.clip(FLvalue, -1, 1 );
            FRvalue = Range.clip(FRvalue, -1, 1 );
            BLvalue = Range.clip(BLvalue, -1, 1 );
            BRvalue = Range.clip(BRvalue, -1, 1 );

            robot.frontLeft.setPower(FLvalue);
            robot.frontRight.setPower(FRvalue);
            robot.backLeft.setPower(BLvalue);
            robot.backRight.setPower(BRvalue);

/*
            DriveB(1,1000);
            if(sensorColor.Red()< #){
                FLvalue.setPower(P);
                Thread.Sleep(500);
            }
            else{
                FRvalue.SetPower(P);
                Thread.Sleep(500);
            }
*/
            DriveF(1, 3000);
            stopDriving();
        }
        public void DriveF(double P, long T) throws InterruptedException
    {
        FRvalue.setPower(P);
        FLvalue.setPower(P);
        BLvalue.setPower(P);
        BRvalue.setPower(P);
        Thread.sleep(T);

    }
        public void stopDriving(double P)
        FRvalue.setPower(0);
        FLvalue.setPower(0);
        BLvalue.setPower(0);
        BRvalue.setPower(0);
    }
    public void DriveB(double P, long T) throws Interrupted Exception
        FRvalue.setPower(-P);
        FLvalue.setPower(-P);
        BLvalue.setPower(-P);
        BRvalue.setPower(-P);
        Thread.Sleep(T);
}


