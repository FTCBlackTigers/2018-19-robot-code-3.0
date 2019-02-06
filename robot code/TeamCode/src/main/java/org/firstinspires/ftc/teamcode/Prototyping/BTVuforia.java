/* Copyright (c) 2018 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import java.util.ArrayList;
import java.util.List;

public class BTVuforia {

    public static BTVuforia singalton = new BTVuforia();

    private static final String VUFORIA_KEY = "private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT";

    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private static final float CAMERA_FORWARD_DISPLACEMENT  = 0;
    private static final float CAMERA_VERTICAL_DISPLACEMENT = mmTargetHeight;
    private static final float CAMERA_LEFT_DISPLACEMENT  = 0;
    private static final float CAMERA_LONG_AXIS_ROT = 90;
    private static final float CAMERA_SHORT_AXIS_ROT = 0;
    private static final float CAMERA_DEPTH_AXIS_ROT = -90;

    private VuforiaLocalizer vuforia;
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    private OpenGLMatrix lastPosition = null;

    public BTVuforia() {}

    public void init(HardwareMap hardwareMap)
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = FRONT;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix//blue rover position
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix//red footprint position
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix//front crater posision
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix//back space position
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix//phone position on the robot
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_LONG_AXIS_ROT,CAMERA_SHORT_AXIS_ROT, CAMERA_DEPTH_AXIS_ROT));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
        targetsRoverRuckus.activate();
    }

    //is any picture is visible if so returns which one
    private VuforiaTrackable getVisibleTarget() {
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                return trackable;
            }
        }
        return null;
    }

    //returns the full position and rotation of the robot
    private OpenGLMatrix getRobotPosition() {
        VuforiaTrackable visibalePicture = getVisibleTarget();
        if (visibalePicture != null) {
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) visibalePicture.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastPosition = robotLocationTransform;
            }
        }
        return lastPosition;
    }

    //returns visible picture name
    public String getVisableTargetName()
    {
        VuforiaTrackable trackable = getVisibleTarget();
        if(trackable != null)
        {
            return trackable.getName();
        }
        return null;
    }

    //returns the X value of the robot position
    public Float getXpos()
    {
        OpenGLMatrix robotPos = getRobotPosition();
        if(robotPos != null)
        {
            return robotPos.getTranslation().get(0);
        }
        return  null;
    }

    //returns the Y value of the robot position
    public Float getYpos()
    {
        OpenGLMatrix robotPos = getRobotPosition();
        if(robotPos != null)
        {
            return robotPos.getTranslation().get(1);
        }
        return  null;
    }

    //returns the rotation of the robot around the X axis
    public Float getRotation()
    {
        Orientation rotation = Orientation.getOrientation(getRobotPosition(), EXTRINSIC, XYZ, DEGREES);
        if(rotation != null)
        {
            return rotation.thirdAngle;
        }
        return  null;
    }

    //returns the distance from the visible picture wall
    public Float getDistanceFromWall()
    {
        Float robotPos = null;
        Float picPos = null;

        VuforiaTrackable trackable = getVisibleTarget();
        if(trackable != null) {
            switch (trackable.getName()) {
                case "Red-Footprint":
                case "Blue-Rover":
                    robotPos = getYpos();
                    picPos = trackable.getLocation().getTranslation().get(1);
                    break;

                case "Front-Craters":
                case "Back-Space":
                    robotPos = getXpos();
                    picPos = trackable.getLocation().getTranslation().get(0);
                    break;
            }
            if (robotPos != null) {
                return Math.abs(robotPos - picPos);
            }
        }
        return  null;
    }

    @Override
    public String toString() {
        OpenGLMatrix position = getRobotPosition();
        if(position != null)
        {
            VectorF translation = position.getTranslation();
            Orientation rotation = Orientation.getOrientation(position, EXTRINSIC, XYZ, DEGREES);

            return "Visible Target: " + getVisableTargetName() + "\n" +
                    "distance from visible wall: " + getDistanceFromWall() + "\n" +
                    "Pos (mm) {X, Y, Z} = " + translation.get(0) + translation.get(1) + translation.get(2) + "\n " +
                    "Rot (deg) {Roll, Pitch, Heading} = " + Math.round(rotation.firstAngle) + ", " + Math.round(rotation.secondAngle) + ", " + Math.round(rotation.thirdAngle);
        }
        else
        {
            return "no target is visible";
        }
    }
}
