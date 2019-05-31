package org.firstinspires.ftc.teamcode.Autonomous.UsingWebCam.Creater;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotSystems.Climbing;
import org.firstinspires.ftc.teamcode.RobotSystems.Drive;
import org.firstinspires.ftc.teamcode.Util.GoldRecognation;

@Autonomous(name = "DepotToCreaterCam", group = "Depot")
public class depotToCreaterCam extends DepotCam {
    public void endAuto(GoldRecognation.MineralPos goldPos){
        switch (goldPos) {
            case RIGHT:
                robot.climbing.moveLiftAuto(Climbing.Height.DRIVE_POS);
                robot.drive.turnByGyroAbsolut(-40, 3);
                robot.drive.driveByEncoder(30, 0.5, Drive.Direction.BACKWARD, 3);
                robot.drive.turnByGyroAbsolut(60, 3);
                robot.drive.driveByEncoder(140, 0.5, Drive.Direction.FORWARD, 3);
                robot.collectAuto();
                break;
            case CENTER:
            case UNKNOWN:
                robot.climbing.moveLiftAuto(Climbing.Height.DRIVE_POS);
                robot.drive.turnByGyroAbsolut(-35, 3);
                robot.drive.driveByEncoder(28,0.5, Drive.Direction.BACKWARD, 3);
                robot.drive.turnByGyroAbsolut(60,3);
                robot.drive.driveByEncoder(110, 0.5, Drive.Direction.FORWARD, 3);
                robot.collectAuto();
                break;
            case LEFT:
                robot.climbing.moveLiftAuto(Climbing.Height.DRIVE_POS);
                robot.drive.turnByGyroAbsolut(-40, 3);
                robot.drive.driveByEncoder(15, 0.5, Drive.Direction.BACKWARD, 3);
                robot.drive.turnByGyroAbsolut(60, 3);
                robot.drive.driveByEncoder(135, 0.5, Drive.Direction.FORWARD, 3);
                robot.collectAuto();
                break;

        }
    }
}
