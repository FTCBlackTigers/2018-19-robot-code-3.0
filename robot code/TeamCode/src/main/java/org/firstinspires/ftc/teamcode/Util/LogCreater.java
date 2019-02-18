package org.firstinspires.ftc.teamcode.Util;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.wifi.NetworkConnection;
import com.qualcomm.robotcore.wifi.WifiAssistant;
import com.qualcomm.robotcore.wifi.WifiDirectAssistant;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.internal.network.WifiDirectAgent;
import org.firstinspires.ftc.robotcore.internal.network.WifiDirectChannelManager;
import org.firstinspires.ftc.robotcore.internal.network.WifiState;
import org.firstinspires.ftc.teamcode.Prototyping.SensorColor;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.sql.Time;
import java.util.Calendar;
import java.util.Date;
import java.util.Timer;
import java.util.Date;
import java.text.SimpleDateFormat;

public class LogCreater {
    private File log;
    private String PATH = "/sdcard/FIRST/Log-";
    private FileOutputStream outputStream;
    private PrintStream printStream;
    private String fileName;
    private OpMode opMode;

    public LogCreater(String fileName){
        this.fileName = fileName;
    }

    public void init(OpMode opMode) {
        //Calendar.getInstance().getTime();
        try {
            SimpleDateFormat formatter = new SimpleDateFormat("_dd.MM.yyyy_HH:mm:ss");
            Date date = new Date();
            log = new File(PATH + fileName + formatter.format(date) + ".txt");
            outputStream = new FileOutputStream(log);
            printStream = new PrintStream(outputStream);
            printStream.write(" Time, Object, Value, Comments ".getBytes());
            newLine();
            this.opMode = opMode;
        } catch (IOException E) {
        }
    }

    public void writeLog(String object, double value, String comments){
        try {
           printStream.write((opMode.getRuntime() + "," + object + "," + value + "," + comments).getBytes());
           newLine();
        }catch(IOException E){}
    }
    public void newLine(){
        try {
            printStream.write("\n".getBytes());
        }catch (IOException e){}
    }
}
