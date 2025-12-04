package org.firstinspires.ftc.teamcode.apriltagdetection;

import androidx.annotation.NonNull;

/**
 * A class that hold the position in a 3D space along with the rotations around each axis.
 */
public class Pose3D {
    public final double x;
    public final double y;
    public final double z;
    public final double pitch;
    public final double roll;
    public final double yaw;

    /**
     * Creates a Pose3D.
     */
    public Pose3D() {
        x = y = z = pitch = roll = yaw = 0;
    }

    /**
     * Creates a Pose3D.
     * @param x the position on the x axis (lateral)
     * @param y the position on the y axis (forward)
     * @param z the position on the z axis (up)
     * @param pitch the rotation around x axis (is tilted more forward or backwards)
     * @param roll the rotation around y axis (is more upside down or more normal)
     * @param yaw the rotation around z axis (is facing more to the left or to the right)
     */
    public Pose3D(double x, double y, double z, double pitch, double roll, double yaw) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.pitch = pitch;
        this.roll = roll;
        this.yaw = yaw;
    }

    @NonNull
    @Override
    public String toString() {
        return "\nx=" + x +
                "\ny=" + y +
                "\nz=" + z +
                "\npitch=" + pitch +
                "\nroll=" + roll +
                "\nyaw=" + yaw;
    }
}
