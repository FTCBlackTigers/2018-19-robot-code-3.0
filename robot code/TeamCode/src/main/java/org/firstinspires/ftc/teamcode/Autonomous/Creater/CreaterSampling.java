package org.firstinspires.ftc.teamcode.Autonomous.Creater;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotSystems.Climbing;
import org.firstinspires.ftc.teamcode.RobotSystems.Drive;
import org.firstinspires.ftc.teamcode.RobotSystems.Robot;

@Autonomous(name = "CreaterSampling", group = "Tests")
@Disabled
public class CreaterSampling extends LinearOpMode{
    private Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        waitForStart();
        robot.climbing.land();
        //robot.drive.Sampling();
        telemetry.addData("gyro angle: ", robot.drive.getAngle());
        telemetry.update();
        robot.climbing.moveLift(Climbing.Height.DRIVE_POS);
        robot.drive.turnByGyroAbsolut(65);//roatate to the wall
        robot.drive.driveByEncoder(85, 0.3, Drive.Direction.BACKWARD, 2000); //drive to the wall
        robot.drive.turnByGyroAbsolut(150); //rotate to the crater
        robot.drive.driveByEncoder(20, 0.3, Drive.Direction.FORWARD, 2000);//drive to the crater
        robot.drive.driveByEncoder(10, 0.3, Drive.Direction.FORWARD, 2000);
        robot.drive.turnByGyroAbsolut(150);
        robot.drive.driveByEncoder(10, 0.3, Drive.Direction.BACKWARD, 2000);
        sleep(10000);
       // robot.intake.release();
        //sleep(2500);
      //  robot.intake.stopMotor();
        //robot.drive.driveByEncoder(120, 0.5, Drive.Direction.FORWARD, 2000);
    }

}
