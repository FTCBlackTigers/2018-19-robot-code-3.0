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

/**
 * doing Landing,sampling and Team Marker
 * PTS = 70
 * Starting from Depot On The Lander
 * FIRST Autonomous
 */
@Autonomous(name = "DepotCam", group = "Depot")

public class DepotCam extends LinearOpMode {

    protected Robot robot = new Robot();
    protected ElapsedTime runtime = new ElapsedTime();
    protected LogCreater log = new LogCreater("auto");
    protected GoldRecognation recognation = null;

    public void endAuto(GoldRecognation.MineralPos goldPos) {
        if (goldPos == GoldRecognation.MineralPos.LEFT) {
            robot.drive.turnByGyroAbsolut(-45, 10);
            robot.drive.driveByEncoder(100, 0.5, Drive.Direction.FORWARD, 3000);
            robot.climbing.moveAngleAuto(Climbing.Angle.COLLECT);
            robot.intake.collect();
            robot.climbing.moveLiftAuto(Climbing.Height.PUT);
            robot.climbing.moveLiftAuto(Climbing.Height.COLLECT);
            robot.climbing.moveLiftAuto(Climbing.Height.PUT);
        } else {
            robot.drive.driveByEncoder(40, 0.4, Drive.Direction.FORWARD, 3000);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        log.init(this);
        robot.init(hardwareMap, this, log);
        recognation = new GoldRecognation(hardwareMap,this);
        GoldRecognation.MineralPos goldPos = GoldRecognation.MineralPos.UNKNOWN;
        while(!isStarted() && !isStopRequested()){
            goldPos = recognation.getGoldPosUsingCam(log);
            idle();
        }
        // start
        double timeToStop = getRuntime() + 3;
        while (goldPos == GoldRecognation.MineralPos.UNKNOWN && opModeIsActive() && getRuntime() <= timeToStop){
            goldPos = recognation.getGoldPosUsingCam(log);
        }
        robot.land();
        robot.drive.samplingCam(Drive.Side.DEPOT, goldPos);
        //robot.drive.turnByGyroRelative(15, 2);
        //robot.drive.driveByEncoder(10, 0.5, Drive.Direction.BACKWARD, 5);
        robot.intake.injackt();
        sleep(3000);
        robot.intake.stop();



    }
}