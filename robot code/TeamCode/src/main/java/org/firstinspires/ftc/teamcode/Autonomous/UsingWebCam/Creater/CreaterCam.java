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

package org.firstinspires.ftc.teamcode.Autonomous.UsingWebCam.Creater;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotSystems.Climbing;
import org.firstinspires.ftc.teamcode.RobotSystems.Drive;
import org.firstinspires.ftc.teamcode.RobotSystems.Robot;
import org.firstinspires.ftc.teamcode.Util.GlobalVariebels;
import org.firstinspires.ftc.teamcode.Util.GoldRecognation;
import org.firstinspires.ftc.teamcode.Util.LogCreater;


@Autonomous(name = "CreaterCam", group = "CreaterCam")

public class CreaterCam extends LinearOpMode {

    protected Robot robot = new Robot();
    protected ElapsedTime runtime = new ElapsedTime();
    protected LogCreater log = new LogCreater("auto");
    protected GoldRecognation recognation = null;


    @Override
    public void runOpMode() throws InterruptedException {
        log.init(this);
        robot.init(hardwareMap, this, log);
        recognation = new GoldRecognation(hardwareMap,this);
        GoldRecognation.MineralPos goldPos = GoldRecognation.MineralPos.UNKNOWN;
        while(!isStarted()){
            goldPos = recognation.getGoldPosUsingCam(log);
            idle();
            if(isStopRequested()){
                stop();
            }
        }
        /**   started   **/
        while (goldPos == GoldRecognation.MineralPos.UNKNOWN){
            goldPos = recognation.getGoldPosUsingCam(log);
        }
        robot.land();
        robot.drive.samplingCam(Drive.Side.CREATER, goldPos);
        goToCreater(goldPos);


    }

    public void goToCreater(GoldRecognation.MineralPos goldpos){
        int degree = 180;
        switch (goldpos){
            case LEFT:
                degree = 160;
                break;
            case CENTER:
            case UNKNOWN:
                degree = 180;
                break;
            case RIGHT:
                degree = -160;
                break;
        }
        robot.drive.turnByGyroAbsolut(degree,3);

        robot.climbing.moveAngleAuto(Climbing.Angle.COLLECT);
        robot.intake.collect();
        sleep(300);
        robot.climbing.moveLiftAuto(Climbing.Height.PUT);
        robot.climbing.moveLiftAuto(Climbing.Height.COLLECT);
        robot.climbing.moveLiftAuto(Climbing.Height.PUT);


    }
}