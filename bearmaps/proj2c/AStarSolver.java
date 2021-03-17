package bearmaps.proj2c;

import bearmaps.proj2ab.DoubleMapPQ;
import edu.princeton.cs.algs4.Stopwatch;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class AStarSolver<Vertex> implements ShortestPathsSolver<Vertex> {
    private HashMap<Vertex, Vertex> edgeTo;
    private HashMap<Vertex, Double> distTo;
    private DoubleMapPQ<Vertex> fringe; // commen
    private SolverOutcome outcome;
    private Vertex goal;
    private int numDequeued;
    private ArrayList<Vertex> solution;
    private double timeElapsed;

    public AStarSolver(AStarGraph<Vertex> input, Vertex start, Vertex end, double timeout) {
        Stopwatch p = new Stopwatch();
        fringe = new DoubleMapPQ<>();
        distTo = new HashMap<>();
        edgeTo = new HashMap<>();
        solution = new ArrayList<>();
        goal = end;
        numDequeued = 0;
        fringe.add(start, 0);
        distTo.put(start, 0.0);
        solution.add(start);
        outcome = SolverOutcome.UNSOLVABLE;
        while (fringe.size() != 0) {
            Vertex v = fringe.removeSmallest();
            if (v.equals(end)) {
                createSolution();
                break;
            }
            numDequeued += 1;
            if (p.elapsedTime() > timeout * 1000) {
                outcome = SolverOutcome.TIMEOUT;
                break;
            }
            for (WeightedEdge<Vertex> n : input.neighbors(v)) {
                relax(n, input, end);
            }
        }
        timeElapsed = p.elapsedTime();
    }

    private void relax(WeightedEdge<Vertex> n, AStarGraph<Vertex> input, Vertex end) {
        Vertex neighbor = n.to();
        Vertex current = n.from();
        double weight = n.weight();

        if (distTo.get(current) + weight
                < distTo.getOrDefault(neighbor, Double.POSITIVE_INFINITY)) {
            distTo.put(neighbor, distTo.get(current) + weight);
            edgeTo.put(neighbor, current);
            double newDist = distTo.get(neighbor) + input.estimatedDistanceToGoal(neighbor, end);
            if (fringe.contains(neighbor)) {
                fringe.changePriority(neighbor, newDist);
            } else {
                fringe.add(neighbor, newDist);
            }
        }
    }

    private void createSolution() {
        Vertex pathBack = goal;
        while (edgeTo.get(pathBack) != null) {
            solution.add(1, pathBack);
            pathBack = edgeTo.get(pathBack);
        }
        outcome = SolverOutcome.SOLVED;
    }

    public SolverOutcome outcome() {
        return outcome;
    }
    public List<Vertex> solution() {
        if (outcome == SolverOutcome.SOLVED) {
            // list of vertices for solution
            return solution;
        }
        return null;
    }
    public double solutionWeight() {
        return distTo.getOrDefault(goal, 0.0);
    }
    public int numStatesExplored() {
        return numDequeued;
    }
    public double explorationTime() {
        return timeElapsed;
    }
}
