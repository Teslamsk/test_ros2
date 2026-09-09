package org.example.vlp16;

/**
 * Transform of the Velodyne VLP-16 mount on the D500 frame.
 *
 * <p>Lidar coordinate system (velodyne driver): x — forward, y — left, z — up
 * (rotation axis). Frame coordinate system (base): x/y — horizontal, z — vertical
 * (frame rotation axis). Transform consists of two parts:
 * <ol>
     *  <li>fixed mount transform {@link #DEFAULT_MATRIX} (lidar → frame): the rotation axis is
     *      horizontal along frame -X (mirror), and the lidar "forward" points up;</li>
 *  <li>frame rotation by frame angle A (rotation around the vertical Z axis) plus
 *      the center's height {@link #DEFAULT_HEIGHT}.</li>
 * </ol>
 *
 * <p>A single source of truth for the mount geometry: if the geometry "doesn't fit" in
 * RViz/CloudCompare, change DEFAULT_MATRIX / DEFAULT_HEIGHT here (same convention as
 * UP_SIGN / CHANNEL_ANGLES_DEG).
 */
public final class Vlp16Mount {

    /**
     * Default mount (rows = frame axes, columns = lidar axes):
     * <pre>
     *   x_f =  0*x_l +  0*y_l - 1*z_l   (rotation axis z_l → frame -X, mirror)
     *   y_f =  0*x_l + -1*y_l + 0*z_l   (lidar left y_l → frame -Y)
     *   z_f =  1*x_l +  0*y_l + 0*z_l   (lidar forward x_l → up)
     * </pre>
     * Mirrored (det = -1): the axis along the lidar rotation axis is flipped.
     * If "forward" is down or the mirror axis is different — flip the corresponding rows/signs.
     */
    public static final float[][] DEFAULT_MATRIX = {
            {0f, 0f, -1f},
            {0f, -1f, 0f},
            {1f, 0f, 0f},
    };

    /** Height of the lidar center above the frame base, meters. */
    public static final float DEFAULT_HEIGHT = 0.4f;

    private Vlp16Mount() {
    }

    /** Fixed part: lidar coordinate system → frame coordinate system (without frame rotation). */
    public static float[] toFrame(float[] pLidar) {
        return toFrame(pLidar, DEFAULT_MATRIX);
    }

    public static float[] toFrame(float[] pLidar, float[][] m) {
        float x = pLidar[0], y = pLidar[1], z = pLidar[2];
        return new float[]{
                m[0][0] * x + m[0][1] * y + m[0][2] * z,
                m[1][0] * x + m[1][1] * y + m[1][2] * z,
                m[2][0] * x + m[2][1] * y + m[2][2] * z,
        };
    }

    /**
     * World (base) coordinates: mount rotation + frame rotation by frameAngleDeg
     * around the vertical Z axis + center height.
     *
     * @param pLidar        point in the lidar coordinate system {x, y, z}
     * @param frameAngleDeg frame angle (degrees), 0 — frame zero
     */
    public static float[] toWorld(float[] pLidar, float frameAngleDeg) {
        return toWorld(pLidar, frameAngleDeg, DEFAULT_MATRIX, DEFAULT_HEIGHT);
    }

    public static float[] toWorld(float[] pLidar, float frameAngleDeg, float[][] m, float height) {
        float[] f = toFrame(pLidar, m);
        double a = Math.toRadians(frameAngleDeg);
        float cos = (float) Math.cos(a);
        float sin = (float) Math.sin(a);
        return new float[]{
                f[0] * cos - f[1] * sin,
                f[0] * sin + f[1] * cos,
                f[2] + height,
        };
    }
}
