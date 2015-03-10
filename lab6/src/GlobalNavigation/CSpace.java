package GlobalNavigation;

import java.util.ArrayList;
import java.util.List;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Point2D.Double;
import java.awt.Color;

/**
 * Contains methods for configuration spaces
 */
public class CSpace {
    public double robotXShift = 0.0;
    public double robotYShift = 0.0;

    //    We are assuming that the robot rotates at its center of the square approximation
    //    This will almost certainly need to be accounted for as the robot does not turn at its center
    private final double ROBOT_WIDTH = 0.67; // in meters
    private final double ROBOT_HEIGHT = 0.67; // in meters

    private PolygonObstacle RobotPolygon;

    public CSpace() {
        //        Creating the robot polygon
        PolygonObstacle origRobotPolygon = new PolygonObstacle();
        origRobotPolygon.addVertex(0.0, 0.0);
        origRobotPolygon.addVertex(ROBOT_WIDTH, 0.0);
        origRobotPolygon.addVertex(ROBOT_WIDTH, ROBOT_HEIGHT);
        origRobotPolygon.addVertex(0.0, ROBOT_HEIGHT);
        origRobotPolygon.close();

        RobotPolygon = changeOrigin(origRobotPolygon, new Point2D.Double(0.0, 0.0));
    }

    /**
     * Computes the Minkowski sum of two polygons
     * @param poly1 the first polygon
     * @param poly2 the second polygon
     * @return the Minkowski sum of the polygons
     */
    public PolygonObstacle ComputeMinkowskiSum(PolygonObstacle poly1, PolygonObstacle poly2) {
		List<Point2D.Double> points = new ArrayList<Point2D.Double>();

        for (Point2D.Double vertex1 : poly1.getVertices()) {
            for (Point2D.Double vertex2 : poly2.getVertices()) {
				points.add(new Point2D.Double(vertex1.getX() + vertex2.getX(), vertex1.getY() + vertex2.getY()));
            }
        }

		return GeomUtils.convexHull(points);
    }

    /**
     * Rescales the polygon's vertices to be with respect to the new reference point
     * @param origPoly the polygon whose vertices are to be rescaled
     * @param refPoint the new reference point
     * @return the new rescaled polygon
     */
    public PolygonObstacle changeOrigin(PolygonObstacle origPoly, Point2D.Double refPoint) {

        int numVertices = origPoly.getVertices().size();
        double xSum = 0.0;
        double ySum = 0.0;

        //        Computing the centroid of the polygon
        for (Point2D.Double vertex : origPoly.getVertices()) {
            xSum += vertex.getX();
            ySum += vertex.getY();
        }

        xSum /= numVertices;
        ySum /= numVertices;

        //        Computing the difference in the x and y components of the centroid and reference point
        robotXShift = refPoint.getX()-xSum;
        robotYShift = refPoint.getY()-ySum;

        //        Rescaling each vertex to have the reference point as the centroid
        return shiftObs(origPoly, robotXShift, robotYShift);
    }

    /**
     * Shifts all the vertices of a polygon by the designated x and y shifts
     * @param origPoly
     * @param xShift
     * @param yShift
     * @return
     */
    public PolygonObstacle shiftObs(PolygonObstacle origPoly, double xShift, double yShift) {
        PolygonObstacle newPoly = new PolygonObstacle();

        for (Point2D.Double vertex : origPoly.getVertices()) {
            newPoly.addVertex(vertex.getX() + xShift, vertex.getY() + yShift);
        }

        newPoly.close();
		newPoly.color = origPoly.color;

        return newPoly;
    }

    /**
     * Computes the configuration space of an obstacle
     * @param obsPoly the polygon of the obstacle
     * @param RobotPolygon the polygon of the robot (not centered at the reference point)
     * @param refPoint the robot's reference point
     * @return the obstacle configuration space
     */
    public PolygonObstacle obsCSpace(PolygonObstacle obsPoly, PolygonObstacle RobotPolygon, Point2D.Double refPoint, boolean computeRobotPolygon) {
        if (computeRobotPolygon) {
            // Setting the robot at the origin
            RobotPolygon = changeOrigin(RobotPolygon, refPoint);
        }

        //       Do we need to shift the obstacle??
        //        Shifting the obstacle by the same amount the robot polygon is shifted to keep everything the same
        //        PolygonObstacle shiftedObsPoly = shiftObs(obsPoly, robotXShift, robotYShift);

        //        To compute the config space of the obstacle, probably need to have the ref point at origin or else when compute minkowski sum, values may be off
        PolygonObstacle cspaceObs = ComputeMinkowskiSum(RobotPolygon, obsPoly);
		cspaceObs.color = obsPoly.color;
		return cspaceObs;
    }

    /**
     * Computes the configuration space of the provided map.
     * @param polygonMap the map of the environment to generate a configuration space of
     * @return the configuration space obstacles of the map obstacles and the boundaries
     */
    public List<PolygonObstacle> envConfSpace(PolygonMap polyMap){
        List<PolygonObstacle> obsCSpaces = new ArrayList<PolygonObstacle>();

        //        Computed the configuration spaces of the obstacle
        for (PolygonObstacle obstacle : polyMap.getObstacles()) {
            obsCSpaces.add(obsCSpace(obstacle, RobotPolygon, null, false));
        }

        //        build obstacle for the boundaries
		PolygonObstacle bottomBound = new PolygonObstacle();
		PolygonObstacle rightBound = new PolygonObstacle();
		PolygonObstacle topBound = new PolygonObstacle();
		PolygonObstacle leftBound = new PolygonObstacle();
		Rectangle2D.Double envBounds = polyMap.worldRect;

		bottomBound.addVertex(envBounds.getX(), envBounds.getY());
		bottomBound.addVertex(envBounds.getX()+envBounds.getWidth(), envBounds.getY());
		bottomBound.close();
		bottomBound.color = Color.BLACK;

		rightBound.addVertex(envBounds.getX()+envBounds.getWidth(), envBounds.getY());
		rightBound.addVertex(envBounds.getX()+envBounds.getWidth(), envBounds.getY()+envBounds.getHeight());
		rightBound.close();
		rightBound.color = Color.BLACK;

		topBound.addVertex(envBounds.getX()+envBounds.getWidth(), envBounds.getY()+envBounds.getHeight());
		topBound.addVertex(envBounds.getX(), envBounds.getY()+envBounds.getHeight());
		topBound.close();
		topBound.color = Color.BLACK;

		leftBound.addVertex(envBounds.getX(), envBounds.getY()+envBounds.getHeight());
		leftBound.addVertex(envBounds.getX(), envBounds.getY());
		leftBound.close();
		leftBound.color = Color.BLACK;

		obsCSpaces.add(obsCSpace(bottomBound, RobotPolygon, null, false));
		obsCSpaces.add(obsCSpace(rightBound, RobotPolygon, null, false));
		obsCSpaces.add(obsCSpace(topBound, RobotPolygon, null, false));
		obsCSpaces.add(obsCSpace(leftBound, RobotPolygon, null, false));
        // PolygonObstacle boundaryObs = new PolygonObstacle();

        // boundaryObs.addVertex(envBounds.getX(), envBounds.getY());
        // boundaryObs.addVertex(envBounds.getX() + envBounds.getWidth(), envBounds.getY());
        // boundaryObs.addVertex(envBounds.getX() + envBounds.getWidth(), envBounds.getY() + envBounds.getHeight());
        // boundaryObs.addVertex(envBounds.getX(), envBounds.getY() + envBounds.getHeight());
		// boundaryObs.color = Color.BLACK;
        // obsCSpaces.add(obsCSpace(boundaryObs, RobotPolygon, null, false));

        return obsCSpaces;
    }
}
