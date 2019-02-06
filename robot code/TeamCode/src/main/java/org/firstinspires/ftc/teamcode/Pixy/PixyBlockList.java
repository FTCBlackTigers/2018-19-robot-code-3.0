package org.firstinspires.ftc.teamcode.Pixy;

import com.qualcomm.robotcore.util.TypeConversion;

import java.util.Vector;

public class PixyBlockList extends Vector<PixyBlock> {
    public int totalCount;

    public PixyBlockList(byte totalCount) {
        this.totalCount = TypeConversion.unsignedByteToInt(totalCount);
    }
}