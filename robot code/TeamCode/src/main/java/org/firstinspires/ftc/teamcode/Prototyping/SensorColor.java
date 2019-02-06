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

package org.firstinspires.ftc.teamcode.Prototyping;


import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import java.io.*;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;


@TeleOp(name = "Sensor: Color", group = "Sensor")
@Disabled
public class SensorColor extends LinearOpMode {

  ColorSensor colorSensor;
  NormalizedColorSensor normalizedColorSensor;
  private final String FILE_PATH = "org/firstinspires/ftc/teamcode/RGB_Test.txt";
  final double SCALE_FACTOR = 255;
  private java.io.File file = new java.io.File(FILE_PATH);
  FileOutputStream fileOutputStream;
  PrintStream printStream;
  View relativeLayout;
  private DcMotor collectMotor;
  private Servo servo;
  float hsvValues[] = {0F, 0F, 0F};

  @Override

  public void runOpMode() throws InterruptedException {

    // Get a reference to the RelativeLayout so we can later change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
    collectMotor = hardwareMap.get(DcMotor.class, "collectMotor");
    collectMotor.setDirection(DcMotor.Direction.FORWARD);
    normalizedColorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
    servo = hardwareMap.get(Servo.class, "Servo");
    servo.setPosition(0.20);

    waitForStart();
    try {
      File file = new File("/sdcard/FIRST/RGB_Test.txt");
      fileOutputStream = new FileOutputStream(file);
      printStream = new PrintStream(fileOutputStream);

      printStream.write("Time | Hue | S_Value | Value \n".getBytes());

      while (opModeIsActive()) {
        collectMotor.setPower(0.8);
        NormalizedRGBA colors = normalizedColorSensor.getNormalizedColors();

        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsvValues);

        printStream.write((((String) (time + "|" + hsvValues[0] + "| " + hsvValues[1] + "| "  + hsvValues[2]) + "\n")).getBytes());
        telemetry.addLine(time + "|" + colors.red + "|" + colors.green + "|" + colors.blue + "\n");
        telemetry.addLine("Red : " + colorSensor.red() + "| green : " + colorSensor.green() + "| blue : "  + colorSensor.blue());
        telemetry.addLine("Hue : " + hsvValues[0] + "| s : " + hsvValues[1] + "| value : "  + hsvValues[2]);
        telemetry.update();

       sleep(200);
      }

      collectMotor.setPower(0);
      printStream.close();
      fileOutputStream.close();

    } catch (IOException e) {
    } finally {


      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.WHITE);
        }
      });
    }

  }

  protected void runSample() throws InterruptedException {

    // values is a reference to the hsvValues array.
    float[] hsvValues = new float[3];
    final float values[] = hsvValues;

    // bPrevState and bCurrState keep track of the previous and current state of the button
    boolean bPrevState = false;
    boolean bCurrState = false;

    // Get a reference to our sensor object.
    //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

    // If possible, turn the light on in the beginning (it might already be on anyway,
    // we just make sure it is if we can).
    if (colorSensor instanceof SwitchableLight) {
      ((SwitchableLight) colorSensor).enableLight(true);
    }

    // Wait for the start button to be pressed.
    waitForStart();

    // Loop until we are asked to stop
    while (opModeIsActive()) {
      // Check the status of the x button on the gamepad
      bCurrState = gamepad1.x;

      // If the button state is different than what it was, then act
      if (bCurrState != bPrevState) {
        // If the button is (now) down, then toggle the light
        if (bCurrState) {
          if (colorSensor instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight) colorSensor;
            light.enableLight(!light.isLightOn());
          }
        }
      }
      bPrevState = bCurrState;

      // Read the sensor
      NormalizedRGBA colors = normalizedColorSensor.getNormalizedColors();

      /** Use telemetry to display feedback on the driver station. We show the conversion
       * of the colors to hue, saturation and value, and display the the normalized values
       * as returned from the sensor.
       * @see <a href="http://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html">HSV</a>*/

      Color.colorToHSV(colors.toColor(), hsvValues);
      telemetry.addLine()
              .addData("H", "%.3f", hsvValues[0])
              .addData("S", "%.3f", hsvValues[1])
              .addData("V", "%.3f", hsvValues[2]);
      telemetry.addLine()
              .addData("a", "%.3f", colors.alpha)
              .addData("r", "%.3f", colors.red)
              .addData("g", "%.3f", colors.green)
              .addData("b", "%.3f", colors.blue);

      /** We also display a conversion of the colors to an equivalent Android color integer.
       * @see Color */


      /*telemetry.addLine("raw Android color: ")
              .addData("a", "%02x", Color.alpha(color))
              .addData("r", "%02x", Color.red(color))
              .addData("g", "%02x", Color.green(color))
              .addData("b", "%02x", Color.blue(color));

      // Balance the colors. The values returned by getColors() are normalized relative to the
      // maximum possible values that the sensor can measure. For example, a sensor might in a
      // particular configuration be able to internally measure color intensity in a range of
      // [0, 10240]. In such a case, the values returned by getColors() will be divided by 10240
      // so as to return a value it the range [0,1]. However, and this is the point, even so, the
      // values we see here may not get close to 1.0 in, e.g., low light conditions where the
      // sensor measurements don't approach their maximum limit. In such situations, the *relative*
      // intensities of the colors are likely what is most interesting. Here, for example, we boost
      // the signal on the colors while maintaining their relative balance so as to give more
      // vibrant visual feedback on the robot controller visual display.
      float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
      colors.red   /= max;
      colors.green /= max;
      colors.blue  /= max;
      color = colors.toColor();

      telemetry.addLine("normalized color:  ")
              .addData("a", "%02x", Color.alpha(color))
              .addData("r", "%02x", Color.red(color))
              .addData("g", "%02x", Color.green(color))
              .addData("b", "%02x", Color.blue(color));
      telemetry.update();

      // convert the RGB values to HSV values.
      Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);

      // change the background color to match the color detected by the RGB sensor.
      // pass a reference to the hue, saturation, and value array as an argument
      // to the HSVToColor method.
      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
        }
      });
      */
    }
  }
}

