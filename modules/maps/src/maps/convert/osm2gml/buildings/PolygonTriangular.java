package maps.convert.osm2gml.buildings;

import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.misc.geometry.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class PolygonTriangular {

    public static List<List<Point2D>> triangulate(final List<Point2D> originalVertices) {
        final List<List<Point2D>> triangles = new ArrayList<>();

        // ...
        if (originalVertices == null || originalVertices.size() < 3) return triangles;

        // ...
        final List<Point2D> vertices = new ArrayList<>(originalVertices);
        if (vertices.getFirst().equals(vertices.getLast())) vertices.removeLast();

        // ...
        if (vertices.size() == 3) {
            triangles.add(vertices);
            return triangles;
        }

        final boolean isCCW = GeometryTools2D.isCounterClockwise(vertices);
        final int maxIterations = vertices.size() * 2;
        int iterations = 0;
        // WIP:...

        return triangles;
    }

    // Determines whether three consecutive vertices from an "ear" of the polygon.
    private static boolean isEar(
            final Point2D p1, final Point2D p2, final Point2D p3, List<Point2D> polygon, boolean isCCW) {

        // ...
        final Vector2D v1    = p2.minus(p1);
        final Vector2D v2    = p3.minus(p2);
        final double cross   = v1.cross(v2);
        final double epsilon = 1e-8;
        if (isCCW && cross <= epsilon) return false;
        if (!isCCW && -epsilon <= cross) return false;

        // ...
        for (final Point2D pt : polygon) {
            if (pt.equals(p1) || pt.equals(p2) || pt.equals(p3)) continue;
            if (GeometryTools2D.isPointInTriangle(pt, p1, p2, p3)) return false;
        }

        return true;
    }

}
