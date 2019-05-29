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
        robot.drive.turnByGyroAbsolut(140, 3);
        robot.drive.driveByEncoder(85, 0.5, Drive.Direction.FORWARD, 5);
        robot.drive.turnByGyroAbsolut(135, 3);
        robot.collectAuto();
    }
    @Override
    public void goToDepot(GoldRecognation.MineralPos goldPos) {
        switch (goldPos) {
            case LEFT:
                //ToDO: Add backward drive to prevent touching the ball
                robot.drive.turnByGyroAbsolut(110, 3);
                robot.drive.driveByEncoder(55, 0.5, Drive.Direction.BACKWARD, 3);
                robot.drive.turnByGyroAbsolut(130, 4);
                robot.drive.driveByEncoder(50, 0.5 , Drive.Direction.BACKWARD, 3);
                robot.climbing.moveLiftAuto(Climbing.Height.PUT);

        }
        robot.intake.injackt();
        sleep(3000);
        robot.intake.stop();
    }
}
