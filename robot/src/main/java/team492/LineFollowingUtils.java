package team492;

import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import java.util.LinkedList;
import org.opencv.calib3d.Calib3d;
//import org.junit.*;

public class LineFollowingUtils
{
    private CameraFieldOfView cfov;

    public LineFollowingUtils()
    {
        Point tL = new Point(RobotInfo.PIXY2_TOP_LEFT_X, RobotInfo.PIXY2_TOP_LEFT_Y);
        Point tR = new Point(RobotInfo.PIXY2_TOP_RIGHT_X, RobotInfo.PIXY2_TOP_RIGHT_Y);
        Point bL = new Point(RobotInfo.PIXY2_BOTTOM_LEFT_X, RobotInfo.PIXY2_BOTTOM_LEFT_Y);
        Point bR = new Point(RobotInfo.PIXY2_BOTTOM_RIGHT_X, RobotInfo.PIXY2_BOTTOM_RIGHT_Y);
        cfov = new CameraFieldOfView(tL, tR, bL, bR, RobotInfo.PIXY2_LINE_TRACKING_WIDTH,
            RobotInfo.PIXY2_LINE_TRACKING_HEIGHT);
    }

    public double getTurnDegrees(double angle)
    {
        return angle - 90.0;
    }

    public double getAngle(RealWorldPair origin, RealWorldPair p2)
    {
        double dx = p2.getXLength() - origin.getXLength();
        double dy = p2.getYLength() - origin.getYLength();

        double theta = Math.atan2(dy, dx);
        theta = Math.toDegrees(theta);
        theta = (theta + 360.0) % 360.0;
        return theta;
    }

    /**
     * This function creates a very crude estimation of an object's position in
     * real-world units.
     * 
     * @param x0
     *                              the x-coordinate of a pair on the camera
     *                              plane. (x: right positive)
     * @param y0
     *                              the y-coordinate of a pair on the camera
     *                              plane: (y: down positive)
     * @param                   widthCoefficient:
     *                              the width coefficient, the length of the
     *                              camera view width in inches.
     * @param heightCoefficient
     *                              the height coefficient, the height of the
     *                              camera view width in inches.
     * @return a RealWorldPair instance of the approximate scaled real world
     *         location of the objects at the coordinates (x0, y0)
     */
    public RealWorldPair getRWPCrude(int x0, int y0, double widthCoefficient, double heightCoefficient)
    {
        double xLength = widthCoefficient * ((double) x0 / (double) RobotInfo.PIXY2_LINE_TRACKING_WIDTH);
        double yLength = heightCoefficient
            * ((double) (RobotInfo.PIXY2_LINE_TRACKING_HEIGHT - y0) / (double) RobotInfo.PIXY2_LINE_TRACKING_HEIGHT);
        return new RealWorldPair(xLength, yLength);
    }

    /**
     * This function creates an estimation of an object's position in real-world
     * units, using a homography matrix
     * 
     * @param x0
     *               the x-coordinate of a pair on the camera plane. (x: right
     *               positive)
     * @param y0
     *               the y-coordinate of a pair on the camera plane: (y: down
     *               positive)
     * @return a RealWorldPair instance of the approximate scaled real world
     *         location of the objects at the coordinates (x0, y0)
     */
    public RealWorldPair getRWP(int x0, int y0)
    {
        // TODO: Test this implementation
        Point point = cfov.GetRealWorldCoords(new Point(x0, y0));
        return new RealWorldPair(point.x, point.y);
    }

    public static class RealWorldPair
    {
        private double xLength;
        private double yLength;

        public RealWorldPair(double xLength, double yLength)
        {
            this.xLength = xLength;
            this.yLength = yLength;
        }

        public double getXLength()
        {
            return xLength;
        }

        public double getYLength()
        {
            return yLength;
        }
    }

    public static class CameraFieldOfView
    {
        // X,Y robot coordinates of the corners of the image. we're assuming no
        // lens distortion
        private Point topLeft;
        private Point topRight;
        private Point bottomLeft;
        private Point bottomRight;
        private double xResolution;
        private double yResolution;
        // homography matrix mapping image corners to real-world coordinates.
        private Mat H;

        public CameraFieldOfView(
            Point topLeft,
            Point topRight,
            Point bottomLeft,
            Point bottomRight,
            double xResolution,
            double yResolution
        ) {
            this.topLeft = topLeft;
            this.topRight = topRight;
            this.bottomLeft = bottomLeft;
            this.bottomRight = bottomRight;
            this.xResolution = xResolution;
            this.yResolution = yResolution;

            // find the 3x3 homography matrix H 
            LinkedList<Point> objList = new LinkedList<Point>();
            LinkedList<Point> sceneList = new LinkedList<Point>();

            objList.add(topLeft);
            sceneList.add(new Point(0, 0));
            objList.add(topRight);
            sceneList.add(new Point(this.xResolution, 0));
            objList.add(bottomLeft);
            sceneList.add(new Point(0, this.yResolution));
            objList.add(bottomRight);
            sceneList.add(new Point(this.xResolution, this.yResolution));

            MatOfPoint2f obj = new MatOfPoint2f();
            obj.fromList(objList);

            MatOfPoint2f scene = new MatOfPoint2f();
            scene.fromList(sceneList);

            // find the homography from scene (image coords) to obj (world
            // coords). note this relative to a scale factor. hopefully s=1 
            // will Just Work.
            this.H = Calib3d.findHomography(scene, obj);
        }

        // get the real world coordinates of a image (x,y) point.
        public Point GetRealWorldCoords(Point p)
        {
            // todo: verify
            Mat pMat = Mat.ones(new Size(3, 1), CvType.CV_32F);
            pMat.put(0, 0, p.x);
            pMat.put(1, 0, p.y);
            Mat result = new Mat();
            Core.gemm(this.H, pMat, 1.0, new Mat(), 0, result);
            return new Point(
                result.get(0, 0)[0] / result.get(0, 2)[0],
                result.get(0, 1)[0] / result.get(0, 2)[0]); // Result has to be scaled by Z.                                                                                                           // Z.
        }

        
        public static void Test() throws Exception
        {
            CameraFieldOfView fov = new CameraFieldOfView(new Point(-2, 2), new Point(2, 2), new Point(-1, 1),
                new Point(1, 1), 320, 240);

            Point[] testIn = new Point[4];
            Point[] testOut = new Point[4];

            testIn[0] = new Point(0, 0);
            testOut[0] = fov.topLeft;

            testIn[1] = new Point(fov.xResolution, 0);
            testOut[1] = fov.topRight;

            testIn[2] = new Point(0, fov.yResolution);
            testOut[2] = fov.bottomLeft;

            testIn[3] = new Point(fov.xResolution, fov.yResolution);
            testOut[3] = fov.bottomRight;

            boolean success = true;
            int i;
            for (i = 0; i < testIn.length; i++)
            {
                Point pOut = fov.GetRealWorldCoords(testIn[i]);
                double dx = pOut.x - testOut[i].x;
                double dy = pOut.y - testOut[i].y;
                double distance = Math.sqrt(dx * dx + dy * dy);
                success = success && distance < 1e-5;
            }
            if (!success) {
                throw new Exception("Failed homography test");
            }

        }
    }
}