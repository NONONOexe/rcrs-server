package maps.convert.osm2gml;

import java.awt.geom.Rectangle2D;
import java.util.*;

import rescuecore2.misc.geometry.Point2D;
import rescuecore2.misc.geometry.Line2D;
import rescuecore2.misc.geometry.GeometryTools2D;
//import rescuecore2.log.Logger;

import maps.convert.ConvertStep;

/**
   This step splits any edges that intersect.
*/
public class SplitIntersectingEdgesStep extends ConvertStep {
    private final TemporaryMap map;
    private int splitCount;
    private Set<Edge> seen;

    /**
       Construct a SplitIntersectingEdgesStep.
       @param map The TemporaryMap to use.
    */
    public SplitIntersectingEdgesStep(TemporaryMap map) {
        this.map = map;
    }

    @Override
    public String getDescription() {
        return "Splitting intersecting edges";
    }

    @Override
    protected void step() {
        debug.setBackground(ConvertTools.getAllDebugShapes(map));
        splitCount = 0;
        int inspectedCount = 0;
        int pass = 0;

        while (true) {
            pass++;
            setStatus("Inspected " + inspectedCount + " edges and split " + splitCount);

            List<Edge> edgeThisPass = new ArrayList<>(map.getAllEdges());
            if (edgeThisPass.isEmpty()) break;

            // Get bounds directly from the map.
            Rectangle2D bounds = map.getBounds();

            double averageDimension = (bounds.getWidth() + bounds.getHeight()) / 2.0;
            double cellSizeInDegrees = averageDimension / 100.0;

            // Handle cases where the map is tiny to avoid a cell size to zero.
            if (cellSizeInDegrees < 1e-9) {
                cellSizeInDegrees = 1e-9;
            }

            SpatialGrid<Edge> grid = new SpatialGrid<>(bounds, cellSizeInDegrees);
            for (Edge e : edgeThisPass) {
                grid.add(e);
            }

            boolean anySplitInPass = false;
            seen = new HashSet<>();
            setProgressLimit(edgeThisPass.size());

            for (int i = 0; i < edgeThisPass.size(); i++) {
                Edge next = edgeThisPass.get(i);
                Set<Edge> nearbyEdges = grid.getNearbyItems(next);

                if (splitEdgeIfIntersecting(next, nearbyEdges)) {
                    anySplitInPass = true;
                }

                inspectedCount++;
                setProgress(i + 1);
            }

            if (!anySplitInPass) {
                break;
            }
        }

        setStatus("Inspected " + inspectedCount + " edges and split " + splitCount + " times over " + pass + " passes");
    }

    // Check if the target edge intersects with any of the candidate edges and splits them if necessary.
    // The process repeats until no further splits occur for the target edge in a single iteration.
    private boolean splitEdgeIfIntersecting(final Edge targetEdge, final Set<Edge> candidateEdges) {
        // Abort if the target edge has already been deleted from the map,
        // or if we have already processed it.
        if (!map.getAllEdges().contains(targetEdge) || seen.contains(targetEdge)) return false;
        seen.add(targetEdge);

        boolean hasSplitOccurred = false;

        // Keep checking the candidate edges until a full pass completes without any splits.
        // This is necessary because splitting an edge alters the map topology.
        while (true) {
            boolean edgeSplitThisIteration = false;

            for (final Edge candidateEdge : candidateEdges) {
                // Skip comparing the target edge with itself.
                if (candidateEdge.equals(targetEdge)) continue;

                // Skip this candidate if it was already split and removed from the map in a previous
                // iteration (Ghost Edge).
                if (!map.getAllEdges().contains(candidateEdge)) continue;

                // If the target edge itself was split and removed during this loop, stop processing.
                if (!map.getAllEdges().contains(targetEdge)) return hasSplitOccurred;

                boolean didSplit = false;
                final Line2D targetLine    = targetEdge.getLine();
                final Line2D candidateLine = candidateEdge.getLine();

                // Check for overlap if the lines are parallel, otherwise check for standard intersection.
                if (GeometryTools2D.parallel(targetLine, candidateLine)) {
                    if (processParallelLines(targetEdge, candidateEdge)) {
                        didSplit = true;
                    }
                } else {
                    if (checkForIntersection(targetEdge, candidateEdge)) {
                        didSplit = true;
                    }
                }

                // If a split happened, update the flags and break out of the for-loop to restart
                // the candidate check.
                if (didSplit) {
                    edgeSplitThisIteration = true;
                    hasSplitOccurred = true;
                    break;
                }
            }

            // If we checked all valid candidates and no splits occured, we are done with this target edge.
            if (!edgeSplitThisIteration) break;
        }

        return hasSplitOccurred;
    }

    /**
       @return True if e1 was split.
    */
    private boolean processParallelLines(Edge e1, Edge e2) {
        Node e1Start = e1.getStart();
        Node e1End = e1.getEnd();
        Node e2Start = e2.getStart();
        Node e2End = e2.getEnd();

        // If the two parallel lines already share an endpoint, they are considered
        // connected, and we should not attempt to split them further.
        if (e1Start.equals(e2Start) || e1Start.equals(e2End) || e1End.equals(e2Start) || e1End.equals(e2End)) {
            return false; // Already connected, do nothing.
        }

        // Then, check for coordinate proximity (handles distinct nodes at same location)
        if (map.isNear(e1Start.getCoordinates(), e2Start.getCoordinates())
         || map.isNear(e1Start.getCoordinates(), e2End.getCoordinates())
         || map.isNear(e1End.getCoordinates(), e2Start.getCoordinates())
         || map.isNear(e1End.getCoordinates(), e2End.getCoordinates())) {
            return false;
        }

        // Possible cases:
        // Shorter line entirely inside longer
        // Shorter line overlaps longer at longer start
        // Shorter line overlaps longer at longer end
        // Shorter line start point is same as longer start and end point is inside
        // Shorter line start point is same as longer end and end point is inside
        // Shorter line end point is same as longer start and start point is inside
        // Shorter line end point is same as longer end and start point is inside
        Edge shorterEdge = e1;
        Edge longerEdge = e2;
        if (e1.getLine().getDirection().getLength() > e2.getLine().getDirection().getLength()) {
            shorterEdge = e2;
            longerEdge = e1;
        }
        Line2D shorter = shorterEdge.getLine();
        Line2D longer = longerEdge.getLine();
        boolean shortStartLongStart = shorterEdge.getStart() == longerEdge.getStart();
        boolean shortStartLongEnd = shorterEdge.getStart() == longerEdge.getEnd();
        boolean shortEndLongStart = shorterEdge.getEnd() == longerEdge.getStart();
        boolean shortEndLongEnd = shorterEdge.getEnd() == longerEdge.getEnd();
        boolean startInside = !shortStartLongStart && !shortStartLongEnd && GeometryTools2D.contains(longer, shorter.getOrigin());
        boolean endInside = !shortEndLongStart && !shortEndLongEnd && GeometryTools2D.contains(longer, shorter.getEndPoint());

        if (startInside && endInside) {
            processInternalEdge(shorterEdge, longerEdge);
            return true;
        }
        else if (startInside) {
            // Either full overlap or coincident end point
            if (shortEndLongStart) {
                processCoincidentNode(shorterEdge, longerEdge, shorterEdge.getEnd());
                return true;
            }
            else if (shortEndLongEnd) {
                processCoincidentNode(shorterEdge, longerEdge, shorterEdge.getEnd());
                return true;
            }
            else {
                // Full overlap
                processOverlap(shorterEdge, longerEdge);
                return true;
            }
        }
        else if (endInside) {
            // Either full overlap or coincident end point
            if (shortStartLongStart) {
                processCoincidentNode(shorterEdge, longerEdge, shorterEdge.getStart());
                return true;
            }
            else if (shortStartLongEnd) {
                processCoincidentNode(shorterEdge, longerEdge, shorterEdge.getStart());
                return true;
            }
            else {
                // Full overlap
                processOverlap(shorterEdge, longerEdge);
                return true;
            }
        }
        return false;
    }

    /**
       @return true if first is split.
    */
    private boolean checkForIntersection(Edge first, Edge second) {
        Point2D intersection = GeometryTools2D.getSegmentIntersectionPoint(first.getLine(), second.getLine());

        if (intersection == null) {
            // Maybe the intersection is within the map's "nearby" tolerance?
            intersection = Objects.requireNonNull(GeometryTools2D.getIntersectionPoint(first.getLine(), second.getLine()));

            // Was this a near miss?
            if (map.isNear(intersection, first.getStart().getCoordinates()) || map.isNear(intersection, first.getEnd().getCoordinates())) {
                // Check that the intersection is actually somewhere on the second segment
                double d = second.getLine().getIntersection(first.getLine());
                if (d < 0 || d > 1) {
                    // Nope. Ignore it.
                    return false;
                }
            }
            else if (map.isNear(intersection, second.getStart().getCoordinates()) || map.isNear(intersection, second.getEnd().getCoordinates())) {
                // Check that the intersection is actually somewhere on the first line segment
                double d = first.getLine().getIntersection(second.getLine());
                if (d < 0 || d > 1) {
                    // Nope. Ignore it.
                    return false;
                }
            }
            else {
                // Not a near miss.
                return false;
            }
        }

        // If the intersection point is very close to an existing endpoint of either line.
        if (map.isNear(intersection, first.getStart().getCoordinates())
         || map.isNear(intersection, first.getEnd().getCoordinates())
         || map.isNear(intersection, second.getStart().getCoordinates())
         || map.isNear(intersection, second.getEnd().getCoordinates())) {
            return false; // Already connected at an endpoint, no split needed.
        }

        Node n = map.getNode(intersection);
        // Split the two edges into 4 (maybe)
        // Was the first edge split?
        boolean splitFirst = !n.equals(first.getStart()) && !n.equals(first.getEnd());
        boolean splitSecond = !n.equals(second.getStart()) && !n.equals(second.getEnd());

        if (splitFirst) {
            map.splitEdge(first, n);
            splitCount++;
        }
        if (splitSecond) {
            map.splitEdge(second, n);
            splitCount++;
        }

        return splitFirst || splitSecond;
    }

    // Splits the longer edge into chunks using the endpoints of the internal shorter edge.
    private void processInternalEdge(final Edge shorterEdge, final Edge longerEdge) {
        final double t1 = GeometryTools2D.positionOnLine(longerEdge.getLine(), shorterEdge.getLine().getOrigin());
        final double t2 = GeometryTools2D.positionOnLine(longerEdge.getLine(), shorterEdge.getLine().getEndPoint());

        final Node firstCutPoint = (t1 < t2) ? shorterEdge.getStart() : shorterEdge.getEnd();
        final Node secondCutPoint = (t1 < t2) ? longerEdge.getEnd() : longerEdge.getStart();

        // Check validity to prevent zero-length edges.
        final boolean isFirstValid =
                !firstCutPoint.equals(longerEdge.getStart()) && !firstCutPoint.equals(longerEdge.getEnd());
        final boolean isSecondValid =
                !secondCutPoint.equals(longerEdge.getStart()) && !secondCutPoint.equals(longerEdge.getEnd());

        if (isFirstValid && isSecondValid) {
            map.splitEdge(longerEdge, firstCutPoint, secondCutPoint);
            splitCount += 2;
        } else if (isFirstValid) {
            map.splitEdge(longerEdge, firstCutPoint);
            splitCount++;
        } else if (isSecondValid) {
            map.splitEdge(longerEdge, secondCutPoint);
            splitCount++;
        }
    }

    // Splits the longer edge at the non-shared node of the shorter edge.
    private void processCoincidentNode(
            final Edge shorterEdge, final Edge longerEdge, final Node sharedNode) {
        // Find the node of the shorter edge that is not shared with the longer edge.
        final Node cutPoint = sharedNode.equals(shorterEdge.getStart()) ?
                shorterEdge.getEnd() : shorterEdge.getStart();

        // Prevent zero-length edges: ensure the cut point isn't already an endpoint
        // of the longer edge.
        if (!cutPoint.equals(longerEdge.getStart()) && !cutPoint.equals(longerEdge.getEnd())) {
            map.splitEdge(longerEdge, cutPoint);
            splitCount++;
        }
    }

    // Splits both edges where they partially overlap.
    private void processOverlap(final Edge shorterEdge, final Edge longerEdge) {
        final Node shorterCutPoint = GeometryTools2D.contains(shorterEdge.getLine(), longerEdge.getLine().getOrigin()) ?
                longerEdge.getStart() : longerEdge.getEnd();
        final Node longerCutPoint = GeometryTools2D.contains(longerEdge.getLine(), shorterEdge.getLine().getOrigin()) ?
                shorterEdge.getStart() : shorterEdge.getEnd();

        // Prevent zero-length edges for the shorter edge.
        if (!shorterCutPoint.equals(shorterEdge.getStart()) && !shorterCutPoint.equals(shorterEdge.getEnd())) {
            map.splitEdge(shorterEdge, shorterCutPoint);
            splitCount++;
        }

        // Prevent zero-length edges for the longer edge.
        if (!longerCutPoint.equals(longerEdge.getStart()) && !longerCutPoint.equals(longerEdge.getEnd())) {
            map.splitEdge(longerEdge, longerCutPoint);
            splitCount++;
        }
    }
}
