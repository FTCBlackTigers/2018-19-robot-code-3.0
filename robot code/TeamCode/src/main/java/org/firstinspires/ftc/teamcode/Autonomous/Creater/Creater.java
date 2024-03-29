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

package org.firstinspires.ftc.teamcode.Autonomous.Creater;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotSystems.Climbing;
import org.firstinspires.ftc.teamcode.RobotSystems.Drive;
import org.firstinspires.ftc.teamcode.RobotSystems.Robot;
import org.firstinspires.ftc.teamcode.Util.GlobalVariebels;
import org.firstinspires.ftc.teamcode.Util.GoldRecognation;
import org.firstinspires.ftc.teamcode.Util.LogCreater;

/**
 * doing Landing, sampling and Parking.
 * PTS=65
 * Starting from CreaterCam
 */
@Autonomous(name = "Creater", group = "Crater")
@Disabled

public class Creater extends LinearOpMode {
  protected Robot robot = new Robot();
  protected ElapsedTime runtime = new ElapsedTime();
  protected LogCreater log = new LogCreater("auto");

  @Override
  public void runOpMode() throws InterruptedException {
    log.init(this);
    robot.init(hardwareMap , this, log);
    waitForStart();
    robot.land();
    robot.drive.turnByGyroAbsolut(-9, 10);
    GoldRecognation.MineralPos goldPos = robot.drive.sampling(Drive.Side.CREATER);
    goToDepot(goldPos);
    goToCreater(goldPos);
    GlobalVariebels.liftPosEndAuto = robot.climbing.liftMotorLeft.getCurrentPosition();

  }

  public void goToDepot(GoldRecognation.MineralPos goldPos) {
  }

  public void goToCreater(GoldRecognation.MineralPos goldPos) {
    switch (goldPos){
      case LEFT:
      robot.drive.turnByGyroAbsolut(0, 10);
      robot.drive.driveByEncoder(15, 0.5 , Drive.Direction.FORWARD, 5);
      robot.drive.turnByGyroAbsolut(-100, 3);
      robot.drive.turnByGyroAbsolut(-170, 5);
      robot.drive.driveByEncoder(25, 0.5, Drive.Direction.FORWARD, 3000);
      break;
      case UNKNOWN:
      case CENTER:
        robot.drive.turnByGyroAbsolut(0, 10);
        robot.drive.driveByEncoder(25,0.5, Drive.Direction.FORWARD, 10);
        robot.drive.turnByGyroAbsolut(100,5);
        robot.drive.turnByGyroAbsolut(180,5);
        robot.drive.driveByEncoder(30, 0.5, Drive.Direction.FORWARD, 3000);
        break;
      case RIGHT:
        robot.drive.turnByGyroAbsolut(0, 10);
        robot.drive.driveByEncoder(15, 0.5 , Drive.Direction.FORWARD, 5);
        robot.drive.turnByGyroAbsolut(100, 5);
        robot.drive.turnByGyroAbsolut(170, 5);
        robot.drive.driveByEncoder(10, 0.5, Drive.Direction.FORWARD, 3000);
        break;
    }

    robot.climbing.moveAngleAuto(Climbing.Angle.COLLECT);
    robot.intake.collect();
    robot.climbing.moveLiftAuto(Climbing.Height.PUT);
    robot.climbing.moveLiftAuto(Climbing.Height.COLLECT);
    robot.climbing.moveLiftAuto(Climbing.Height.PUT);
  }
}