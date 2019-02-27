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

import org.firstinspires.ftc.teamcode.RobotSystems.Climbing;
import org.firstinspires.ftc.teamcode.RobotSystems.Drive;
import org.firstinspires.ftc.teamcode.RobotSystems.Robot;
import org.firstinspires.ftc.teamcode.Util.GoldRecognation;

@Autonomous(name = "CreaterToDepot", group = "Crater")
public class CreaterToDepot extends Creater {
    @Override
    public void goToDepot(GoldRecognation.MineralPos goldPos) {
        /*
        if (goldPos == GoldRecognation.MineralPos.RIGHT || goldPos == GoldRecognation.MineralPos.LEFT) {
            robot.drive.driveByEncoder(70, 0.3, Drive.Direction.FORWARD, 3000);
        }
        else {
            robot.drive.driveByEncoder(55, 0.3, Drive.Direction.FORWARD, 3000);
        }
        robot.drive.turnByGyroAbsolut(55, 10);
        robot.drive.driveByEncoder(135, 0.3, Drive.Direction.BACKWARD, 3000);
        robot.drive.turnByGyroAbsolut(133, 10);
        robot.drive.driveByEncoder(150, 0.5, Drive.Direction.BACKWARD, 3000);
        //robot.climbing.moveLiftAuto(Climbing.Height.PUT);
        */
        switch (goldPos){
            case LEFT:
                robot.drive.curvedDrive(60,3,0.5, Drive.Direction.FORWARD, Drive.CurvedDirection.LEFT);
                robot.drive.driveByEncoder(60,0.4, Drive.Direction.BACKWARD,5);
                robot.drive.turnByGyroAbsolut(130, 5);
                robot.drive.driveByEncoder(80,0.5, Drive.Direction.BACKWARD,6);
                break;
            case UNKNOWN:
            case CENTER:
                //robot.drive.curvedDrive(100,10,0.6, Drive.Direction.FORWARD, Drive.CurvedDirection.LEFT);
                robot.drive.driveByEncoder(20,0.4, Drive.Direction.FORWARD,5);
                robot.drive.turnByGyroAbsolut(85, 3);
                robot.drive.driveByEncoder(80/*100*/,0.6, Drive.Direction.BACKWARD,5);
                robot.drive.turnByGyroAbsolut(130, 5);
                robot.drive.driveByEncoder(70,0.7, Drive.Direction.BACKWARD,6);
                break;
            case RIGHT:
                robot.drive.driveByEncoder(10, 0.5, Drive.Direction.FORWARD, 5);
                robot.drive.curvedDrive(100,6,0.5, Drive.Direction.FORWARD, Drive.CurvedDirection.RIGHT);
                robot.drive.driveByEncoder(60,0.4, Drive.Direction.FORWARD,5);
                robot.drive.turnByGyroAbsolut(140, 3);
                robot.drive.driveByEncoder(80,0.5, Drive.Direction.BACKWARD,6);
                break;
        }
        //robot.drive.turnByGyroAbsolut(160, 3);
        robot.intake.injackt();
        sleep(2000);
        robot.intake.stop();
        robot.drive.turnByGyroAbsolut(160, 1);
    }

    @Override
    public void goToCreater(GoldRecognation.MineralPos goldPos) {
        robot.drive.driveByEncoder(110, 1, Drive.Direction.FORWARD, 3);
        robot.climbing.moveAngleAuto(Climbing.Angle.COLLECT);
        robot.intake.collect();
        robot.climbing.moveLiftAuto(Climbing.Height.PUT);
        robot.climbing.moveLiftAuto(Climbing.Height.COLLECT);
        robot.climbing.moveLiftAuto(Climbing.Height.PUT);
    }

}