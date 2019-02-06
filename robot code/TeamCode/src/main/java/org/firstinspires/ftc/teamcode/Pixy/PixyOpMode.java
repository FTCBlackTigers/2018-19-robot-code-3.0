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

package org.firstinspires.ftc.teamcode.Pixy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Pixy simple test", group = "pixy")
@Disabled
public class PixyOpMode extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    I2cDeviceSynch pixy;

    @Override
    public void init() {
        //setting up Pixy to the hardware map
        pixy = hardwareMap.i2cDeviceSynch.get("pixy");

        //setting Pixy's I2C Address
        pixy.setI2cAddress(I2cAddr.create7bit(0x54));

        //setting Pixy's read window. You'll want these exact parameters, and you can reference the
        //SDK Documentation to learn more
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow (1, 26, I2cDeviceSynch.ReadMode.REPEAT);
        pixy.setReadWindow(readWindow);

        //required to "turn on" the device
        pixy.engage();
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("Byte 0", pixy.read8(0));
        telemetry.addData("Byte 1", pixy.read8(1));
        telemetry.addData("Byte 2", pixy.read8(2));
        telemetry.addData("Byte 3", pixy.read8(3));
        telemetry.addData("Byte 4", pixy.read8(4));
        telemetry.addData("Byte 5", pixy.read8(5));
        telemetry.addData("Byte 6", pixy.read8(6));
        telemetry.addData("Byte 7", pixy.read8(7));
        telemetry.addData("Byte 8", pixy.read8(8));
        telemetry.addData("Byte 9", pixy.read8(9));
        telemetry.addData("Byte 10", pixy.read8(10));
        telemetry.addData("Byte 11", pixy.read8(11));
        telemetry.addData("Byte 12", pixy.read8(12));
        telemetry.addData("Byte 13", pixy.read8(13));
        telemetry.update();
    }
}

