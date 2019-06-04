package org.firstinspires.ftc.teamcode.Autonomous.UsingWebCam.Creater;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotSystems.Climbing;
import org.firstinspires.ftc.teamcode.RobotSystems.Drive;
import org.firstinspires.ftc.teamcode.Util.GoldRecognation;

@Autonomous(name = "CreaterToDepotCam", group = "CreaterCam")
public class CreaterToDepotCam extends CreaterCam {
    @Override
    public void goToCreater(GoldRecognation.MineralPos goldpos) {
        robot.climbing.moveLiftAuto(Climbing.Height.DRIVE_POS);
        robot.drive.turnByGyroAbsolut(145, 3);
        robot.drive.driveByEncoder(80, 0.6, Drive.Direction.FORWARD, 5);
        robot.drive.turnByGyroAbsolut(135, 3);
        robot.collectAuto();
    }
    @Override
    public void goToDepot(GoldRecognation.MineralPos goldPos) {
        switch (goldPos) {
            case LEFT:
                robot.drive.driveByEncoder(20, 0.5, Drive.Direction.BACKWARD, 3);
                robot.drive.turnByGyroAbsolut(100, 3);
                robot.drive.driveByEncoder(72, 0.5, Drive.Direction.BACKWARD, 3);
                robot.drive.turnByGyroAbsolut(130, 4);
                robot.drive.driveByEncoder(40, 0.5 , Drive.Direction.BACKWARD, 3);
                break;
            case UNKNOWN:
            case CENTER:
                robot.drive.driveByEncoder(20, 0.5, Drive.Direction.BACKWARD, 3);
                robot.drive.turnByGyroAbsolut(100, 3);
                robot.drive.driveByEncoder(95, 0.5, Drive.Direction.BACKWARD, 3);
                robot.drive.turnByGyroAbsolut(130, 4);
                robot.drive.driveByEncoder(40, 0.5 , Drive.Direction.BACKWARD, 3);

                break;
            case RIGHT:
                robot.drive.driveByEncoder(20, 0.5, Drive.Direction.BACKWARD, 3);
                robot.drive.turnByGyroAbsolut(90, 3);
                robot.drive.driveByEncoder(103, 0.7, Drive.Direction.BACKWARD, 3);
                robot.drive.turnByGyroAbsolut(130, 4);
                robot.drive.driveByEncoder(35, 0.5 , Drive.Direction.BACKWARD, 3);
                break;


        }
        robot.climbing.moveLiftAuto(Climbing.Height.PUT);
        robot.intake.injackt();
        sleep(1500);
        robot.intake.stop();
    }
}
