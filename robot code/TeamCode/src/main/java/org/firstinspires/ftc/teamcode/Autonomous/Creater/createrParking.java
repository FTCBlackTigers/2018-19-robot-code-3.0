package org.firstinspires.ftc.teamcode.Autonomous.Creater;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotSystems.Climbing;
import org.firstinspires.ftc.teamcode.RobotSystems.Drive;
import org.firstinspires.ftc.teamcode.RobotSystems.Robot;
@Autonomous(name = "CreaterParking", group = "autonmous")
@Disabled
public class createrParking extends LinearOpMode{
    protected Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        waitForStart();
        robot.climbing.land();
        robot.climbing.moveLift(Climbing.Height.DRIVE_POS);
        //robot.drive.Sampling();
        robot.drive.turnByGyroRelative(10);
        robot.drive.driveByEncoder(10, 0.4, Drive.Direction.BACKWARD, 1);
        robot.drive.turnByGyroRelative(10);
        robot.drive.driveByEncoder(10, 0.4, Drive.Direction.BACKWARD, 1);

    }
}
