
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Assertions;
import java.util.PriorityQueue;
import java.util.List;
import java.util.LinkedList;
import java.util.NoSuchElementException;


/**
 * This class extends the BaseGraph data structure with additional methods for
 * computing the total cost and list of node data along the shortest path
 * connecting a provided starting to ending nodes. This class makes use of
 * Dijkstra's shortest path algorithm.
 */
public class DijkstraGraph<NodeType, EdgeType extends Number>
    extends BaseGraph<NodeType, EdgeType>
    implements GraphADT<NodeType, EdgeType> {

  /**
   * While searching for the shortest path between two nodes, a SearchNode
   * contains data about one specific path between the start node and another
   * node in the graph. The final node in this path is stored in its node
   * field. The total cost of this path is stored in its cost field. And the
   * predecessor SearchNode within this path is referened by the predecessor
   * field (this field is null within the SearchNode containing the starting
   * node in its node field).
   *
   * SearchNodes are Comparable and are sorted by cost so that the lowest cost
   * SearchNode has the highest priority within a java.util.PriorityQueue.
   */
  protected class SearchNode implements Comparable<SearchNode> {
    public Node node;
    public double cost;
    public SearchNode predecessor;

    public SearchNode(Node node, double cost, SearchNode predecessor) {
      this.node = node;
      this.cost = cost;
      this.predecessor = predecessor;
    }

    public int compareTo(SearchNode other) {
      if (cost > other.cost)
        return +1;
      if (cost < other.cost)
        return -1;
      return 0;
    }
  }

  /**
   * Constructor that sets the map that the graph uses.
   * 
   * @param map the map that the graph uses to map a data object to the node
   *            object it is stored in
   */
  public DijkstraGraph(MapADT<NodeType, Node> map) {
    super(map);
  }

  /**
   * This helper method creates a network of SearchNodes while computing the
   * shortest path between the provided start and end locations. The
   * SearchNode that is returned by this method is represents the end of the
   * shortest path that is found: it's cost is the cost of that shortest path,
   * and the nodes linked together through predecessor references represent
   * all of the nodes along that shortest path (ordered from end to start).
   *
   * @param start the data item in the starting node for the path
   * @param end   the data item in the destination node for the path
   * @return SearchNode for the final end node within the shortest path
   * @throws NoSuchElementException when no path from start to end is found
   *                                or when either start or end data do not
   *                                correspond to a graph node
   */

protected SearchNode computeShortestPath(NodeType start, NodeType end) {

        // Exception if no start or end
        if(!nodes.containsKey(start) || !nodes.containsKey(end)){
            throw new NoSuchElementException("No path found. A start or end node doesn't exist");
        }

        // Exception if start and end are the same
        if(start.equals(end)){
            return new SearchNode(new Node(start), 0, null);
        }

        PriorityQueue<SearchNode> queue = new PriorityQueue<>();
        MapADT<Node, Boolean> visited = new PlaceholderMap<>();
        Node startNode = nodes.get(start);
        SearchNode startPath = new SearchNode(startNode, 0, null);


        queue.add(startPath);

        // While queue is not empty
        while(!queue.isEmpty()){
            SearchNode current = queue.poll(); // removes the front of the queue

            // Skip current node if already been visited
            if (visited.containsKey(current.node)) {
                continue;
            }

            visited.put(current.node, true);

            // Check if current node is the end node
            if (current.node.data.equals(end)) {
                return current;
            }

            // Check neighbors of current node
            for (Edge edge : current.node.edgesLeaving) {
                Node neighbor = edge.successor;
                

                // Add neighbor to queue if not visited
                if (!visited.containsKey(neighbor)) {
                    queue.add(new SearchNode(neighbor, current.cost + edge.data.doubleValue(), current));
                }
                // Exception if edge cost is negative
                if (edge.data.doubleValue() <= 0){
                    throw new NoSuchElementException("Invalid Edge Cost");
                }
            }

        }

        throw new NoSuchElementException("No path exists between  " + start + "to " + end);
    }




  /**
   * Returns the list of data values from nodes along the shortest path
   * from the node with the provided start value through the node with the
   * provided end value. This list of data values starts with the start
   * value, ends with the end value, and contains intermediary values in the
   * order they are encountered while traversing this shorteset path. This
   * method uses Dijkstra's shortest path algorithm to find this solution.
   *
   * @param start the data item in the starting node for the path
   * @param end   the data item in the destination node for the path
   * @return list of data item from node along this shortest path
   */
  public List<NodeType> shortestPathData(NodeType start, NodeType end) {
    SearchNode endNode = computeShortestPath(start, end);
  
    List<NodeType> path = new LinkedList<>();
    SearchNode current = endNode;
    while (current != null) {
      path.add(0, current.node.data);
      current = current.predecessor; 
    }
  
    return path;
  }

  /**
   * Returns the cost of the path (sum over edge weights) of the shortest
   * path freom the node containing the start data to the node containing the
   * end data. This method uses Dijkstra's shortest path algorithm to find
   * this solution.
   *
   * @param start the data item in the starting node for the path
   * @param end   the data item in the destination node for the path
   * @return the cost of the shortest path between these nodes
   */
  public double shortestPathCost(NodeType start, NodeType end) {
    SearchNode endNode = computeShortestPath(start, end);
    return endNode.cost;
  }

  // TODO: implement 3+ tests in step 4.1
 class DijkstraGraphTest {

    @Test
    void testShortestPathData() {
      DijkstraGraph<String, Integer> graph = new DijkstraGraph<>(new PlaceholderMap<>());
      // populate graph

      List<String> expectedPath = List.of("A", "B", "D");
      List<String> actualPath = graph.shortestPathData("A", "D");

      Assertions.assertEquals(expectedPath, actualPath);
    }

    @Test
    void testShortestPathCost() {
      DijkstraGraph<String, Integer> graph = new DijkstraGraph<>(new PlaceholderMap<>());
      // populate graph

      double expectedCost = 15.0;
      double actualCost = graph.shortestPathCost("A", "E");

      Assertions.assertEquals(expectedCost, actualCost);
    }

    @Test
    void testNoPath() {
      DijkstraGraph<String, Integer> graph = new DijkstraGraph<>(new PlaceholderMap<>());
      // populate graph

      Assertions.assertThrows(NoSuchElementException.class, () -> {graph.shortestPathData("A", "F");}); 
    }

  }



}
