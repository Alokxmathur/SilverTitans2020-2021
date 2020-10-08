package org.firstinspires.ftc.teamcode.robot.components.camera;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

public class SkyStoneDetector {
    private Alliance.Color allianceColor;

    public int getValMid() {
        return valMid;
    }

    public int getValLeft() {
        return valLeft;
    }

    public int getValRight() {
        return valRight;
    }

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private int valMid = -1;
    private int valLeft = -1;
    private int valRight = -1;

    private float offsetX;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private float offsetY = .1f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    Mat yCbCrChan2Mat = new Mat();
    Mat thresholdMat = new Mat();
    Mat all = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();
    Mat input = new Mat();

    public SkyStoneDetector(Alliance.Color allianceColor) {
        this.allianceColor = allianceColor;
        this.offsetX = allianceColor == Alliance.Color.BLUE ? .12f : -.09f;
    }
    public Field.SkyStonePosition getSkyStonePosition(Bitmap bitmap)
    {
        contoursList.clear();
        //convert bitmap to Mat
        Utils.bitmapToMat(bitmap, input);
        /*
         * This pipeline finds the contours of yellow blobs such as the Gold Mineral
         * from the Rover Ruckus game.
         */

        //color diff cb.
        //lower cb = more blue = skystone = white
        //higher cb = less blue = yellow stone = grey
        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
        Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

        //b&w
        Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

        //outline/contour
        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        yCbCrChan2Mat.copyTo(all);//copies mat object
        //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours

        float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
        float[] leftPos = {2.5f/8f+offsetX, 4f/8f+offsetY};
        float[] rightPos = {5.5f/8f+offsetX, 4f/8f+offsetY};

        //get values from frame
        double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
        valMid = (int)pixMid[0];

        double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
        valLeft = (int)pixLeft[0];

        double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
        valRight = (int)pixRight[0];

        Field.SkyStonePosition skyStonePosition = Field.SkyStonePosition.TOP;
        if (valMid == 0) {
            skyStonePosition = Field.SkyStonePosition.MIDDLE;
        }
        else {
            if (this.allianceColor == Alliance.Color.BLUE) {
                if (valRight == 0) {
                    skyStonePosition = Field.SkyStonePosition.TOP;
                } else if (valLeft == 0) {
                    skyStonePosition = Field.SkyStonePosition.BOTTOM;
                } else {
                    Match.log("Defaulting Sky stone position = " + skyStonePosition);
                }
            }
            else {
                if (valRight == 0) {
                    skyStonePosition = Field.SkyStonePosition.BOTTOM;
                } else if (valLeft == 0) {
                    skyStonePosition = Field.SkyStonePosition.TOP;
                } else {
                    Match.log("Defaulting Sky stone position = " + skyStonePosition);
                }
            }
        }
        //Match.log("left: " + valLeft + ", mid: " + valMid + ", right: " + valRight);
        return skyStonePosition;
    }
}

