import java.util.*;

public class MainMST {
    /*
     * Viktor Kovalev
     * Task D.
     */
    public static void main(String[] args) throws Exception {
        // Create all variables for Graph and Heap.
        final int size = 2910;
        Graph<Pair, Double> branches = new Graph<>(size);
        HashMap<String, Vertex<Pair>> names = new HashMap<>(size);
        Scanner in = new Scanner(System.in);
        int n = in.nextInt();
        // Array of nodes for heap which is stored vertex.
        ArrayList<Node<Double, Vertex<Pair>>> arrayVertex = new ArrayList<>(size);
        for (int i = 0; i < n; i++) {
            String command = in.next();
            if (command.equals("ADD")) {
                String name = in.next();
                double penalty = in.nextDouble();
                // Create vertex.
                Pair v = new Pair(name, penalty);
                Vertex<Pair> vert = branches.insertVertex(v);
                names.put(name, vert);
                // Add it with needed key into array with nodes.
                if (arrayVertex.size() == 0)
                    arrayVertex.add(new Node<>(0., vert));
                else
                    arrayVertex.add(new Node<>(Double.MIN_VALUE, vert));
            } else if (command.equals("CONNECT")) {
                String name1 = in.next(), name2 = in.next();
                double dist = in.nextDouble();
                // Get vertices from hashMap for creation of edge.
                Vertex<Pair> v = names.get(name1);
                Vertex<Pair> u = names.get(name2);
                dist = dist / (v.getValue().getPenalty() + u.getValue().getPenalty());
                branches.insertEdge(v, u, new Edge<>(dist, v, u));
            } else if (command.equals("PRINT_MIN")) {
                if (arrayVertex.size() == 0) {
                    continue;
                }
                // Just call to Prim's algorithm.
                primsAlgorithm(branches, arrayVertex.get(0).value, arrayVertex);
            }
        }
    }

    // Arrays for Prim's algorithm.
    // For parents in tree.
    static String[] D = new String[3000];
    // For distance.
    static  double[] currentMinDistance = new double[3000];
    // For marks.
    static boolean[] inQueue = new boolean[3000];

    /**
     * Prim's algorithm that finds Minimum Spanning Forest of graph.
     * @param G Graph.
     * @param randVertex Random vertex from graph.
     * @param nodes Array of nodes for heap.
     * @throws Exception
     */
    public static void primsAlgorithm(Graph<Pair, Double> G, Vertex<Pair> randVertex, ArrayList<Node<Double, Vertex<Pair>>> nodes) throws Exception {
        // Priority queue based on Fibonacci heap.
        FibHeap<Double, Vertex<Pair>> Q = new FibHeap<>(Double.MIN_VALUE);
        for (int i = 0; i < G.getSize(); i++) {
            // Fill all fields by default values for node.
            inQueue[i] = true;
            D[i] = "";
            nodes.get(i).mark = false;
            nodes.get(i).children = new CircularQueue<>();
            nodes.get(i).left = nodes.get(i);
            nodes.get(i).right = nodes.get(i);
            nodes.get(i).degree = 0;
            currentMinDistance[i] = Double.MAX_VALUE;
            if (nodes.get(i).value.getValue().name.equals(randVertex.getValue().name)) {
                currentMinDistance[nodes.get(i).value.getId()] = 0;
                nodes.get(i).key = 0.;
            } else {
                nodes.get(i).key = Double.MAX_VALUE;
                // Ignore nodes without edges.
                if (G.degree(nodes.get(i).value) == 0) {
                    inQueue[nodes.get(i).value.getId()] = false;
                    continue;
                }
            }
            // Put all nodes into queue.
            Q.insert(nodes.get(i));
        }
        // String for answer.
        StringBuilder ans = new StringBuilder();
        while (!Q.isEmpty()) {
            // Take minimal node from queue.
            Node<Double, Vertex<Pair>> u = (Node<Double, Vertex<Pair>>) Q.extractMin();
            inQueue[u.value.getId()] = false;
            // Go through all vertices and check for adjacency.
            for (int i = 0; i < G.getSize(); i++){
                // Take link to the edge.
                Edge<Pair, Double> edge = (Edge<Pair, Double>) G.adjacencyMatrix[u.value.getId()][i];
                // Skip it if it null.
                if (edge == null)
                    continue;
                // Take vertex from this edge.
                Node<Double, Vertex<Pair>> v = nodes.get(i);
                // If vertex in queue, and we have better distance through current vertex
                if (inQueue[i] && edge.getDistance().compareTo(currentMinDistance[i]) < 0){
                    // Update for new minimal distance.
                    D[i] = u.value.getValue().name;
                    currentMinDistance[i] = edge.getDistance();
                    // Update key for this node.
                    Q.decreaseKey(v, edge.getDistance());
                }
            }
            // Put into answer string edge with vertex 'u'.
            if (!D[u.value.getId()].equals("")) {
                ans.append(D[u.value.getId()]).append(":").append(u.value.getValue().name).append(" ");
            }
        }
        System.out.println(ans);
    }
}

/**
 * Vertex of Graph.
 * @param <V> Value in vertex.
 */
class Vertex<V> {
    // id in the graph and value.
    private final int id;
    private final V value;

    public Vertex(int id, V value) {
        this.id = id;
        this.value = value;
    }

    public int getId() {
        return id;
    }

    public V getValue() {
        return value;
    }
}

/**
 * Edge in the Graph.
 * @param <V> Value from class Vertex.
 * @param <E> Value for edges.
 */
class Edge<V, E> {
    E distance;
    Vertex<V> from, to;

    public Edge(E distance, Vertex<V> from, Vertex<V> to) {
        this.distance = distance;
        this.from = from;
        this.to = to;
    }

    public E getDistance() {
        return distance;
    }
}

/**
 * Class for representing value for the class Vertex.
 */
class Pair {
    String name;
    double penalty;

    public Pair(String first, double second) {
        this.name = first;
        this.penalty = second;
    }

    public double getPenalty() {
        return penalty;
    }
}

/**
 * Interface for graph.
 * @param <V> Values for vertices.
 * @param <E> Values for the edges.
 */
interface IGraph<V, E> {
    /**
     * Insert new vertex by value.
     * @param v Values of vertex.
     * @return New vertex.
     */
    Vertex<V> insertVertex(V v);

    /**
     * Insert new edge between two vertices.
     * @param from First vertex.
     * @param to Second vertex.
     * @param w Weight of edge.
     * @return New edge.
     */
    Edge<V, E> insertEdge(Vertex<V> from, Vertex<V> to, Edge<V, E> w);

    /**
     * Delete vertex from graph.
     * @param v Vertex for deleting.
     */
    void removeVertex(Vertex<V> v);

    /**
     * Delete edge from graph.
     * @param e Edge for deleting.
     */
    void removeEdge(Edge<V, E> e);

    /**
     * Degree of vertex.
     * @param v Vertex.
     * @return Degree of vertex.
     */
    int degree(Vertex<V> v);

    /**
     * Check two vertices for adjacency.
     * @param v First vertex.
     * @param u Second vertex.
     * @return true - have edge, false - haven't.
     */
    boolean areAdjacent(Vertex<V> v, Vertex<V> u);
}

/**
 * Graph which based on adjacency matrix.
 * @param <V> type of values for vertices.
 * @param <E> type of values for edges.
 */
class Graph<V, E> implements IGraph<V, E> {
    // Degree of each vertex.
    private int[] degree;
    // Size of graph.
    private int size;
    // Capacity of graph.
    private final int capacity;
    private int numberEdges;
    Object[][] adjacencyMatrix;

    public Graph(int capacity) {
        this.degree = new int[capacity];
        for (int i = 0; i < capacity; i++){
            degree[i] = 0;
        }
        this.numberEdges = 0;
        this.size = 0;
        this.capacity = capacity;
        this.adjacencyMatrix = new Object[capacity][capacity];
    }

    /**
     * Method that creates a reference to Vertex element with 'o' value
     * <br/>Time complexity - O(1)
     *
     * @param o Value for vertex.
     * @return Created vertex.
     */
    @Override
    public Vertex<V> insertVertex(V o) {
        return new Vertex<>(size++, o);
    }

    /**
     *
     * @param from First vertex.
     * @param to Second vertex.
     * @param w Weight of edge.
     * @return
     */
    @Override
    public Edge<V, E> insertEdge(Vertex<V> from, Vertex<V> to, Edge<V, E> w) {
        adjacencyMatrix[from.getId()][to.getId()] = w;
        adjacencyMatrix[to.getId()][from.getId()] = w;
        numberEdges++;
        degree[from.getId()]++;
        degree[to.getId()]++;
        return w;
    }

    /**
     *
     * @param v Vertex for deleting.
     */
    @Override
    public void removeVertex(Vertex<V> v) {
        for (int i = 0; i < capacity; i++) {
            adjacencyMatrix[i][v.getId()] = null;
            adjacencyMatrix[v.getId()][i] = null;
        }
        degree[v.getId()] = 0;
    }

    /**
     *
     * @param e Edge for deleting.
     */
    @Override
    public void removeEdge(Edge<V, E> e) {
        adjacencyMatrix[e.from.getId()][e.to.getId()] = null;
        adjacencyMatrix[e.to.getId()][e.from.getId()] = null;
        degree[e.from.getId()]--;
        degree[e.to.getId()]--;
    }

    /**
     * Method that returns the degree of vertex 'v'.
     * <br/>Time complexity: O(size)
     *
     * @param v Considered vertex
     * @return Degree of vertex 'v'
     */
    @Override
    public int degree(Vertex<V> v) {
        return degree[v.getId()];
    }

    /**
     *
     * @param v First vertex.
     * @param u Second vertex.
     * @return
     */
    @Override
    public boolean areAdjacent(Vertex<V> v, Vertex<V> u) {
        return adjacencyMatrix[v.getId()][u.getId()] != null;
    }

    /**
     *
     * @return Size of adjacency matrix.
     */
    public int getSize() {
        return size;
    }
}

/**
 * Node of Priority Queue.
 *
 * @param <K> Key with supporting campareTo() method.
 * @param <V> Value
 */
class Node<K extends Comparable<? super K>, V> {
    // Links to parent, right node and left node.
    public Node<K, V> parent = null;
    public Node<K, V> left;
    public Node<K, V> right;
    // DoubleLinkedList for children of current node.
    public CircularQueue<K, V> children;
    public int degree = 0;
    public boolean mark = false;
    public K key;
    public V value;

    public Node(K key, V value) {
        left = this;
        right = this;
        children = new CircularQueue<>();
        this.key = key;
        this.value = value;
    }

    public Node(Node<K, V> node) {
        this.parent = node.parent;
        this.left = node.left;
        this.right = node.right;
        this.children = new CircularQueue<>();
        this.degree = node.degree;
        this.mark = node.mark;
        this.key = node.key;
        this.value = node.value;
    }

    public String toString() {
        return this.value.toString();
    }
}

/**
 * Double Linked List for root list of fibonacci heap and children of nodes.
 *
 * @param <K> Comparable key.
 * @param <V> Value.
 */
class CircularQueue<K extends Comparable<? super K>, V> {
    // Link to the first element of list.
    Node<K, V> element = null;
    private int size = 0;

    /**
     * Size.
     * <br/>Time complexity - O(1)
     *
     * @return Size of list.
     */
    public int getSize() {
        return size;
    }

    /**
     * Put new node to the list.
     * <br/>Time complexity - O(1)
     *
     * @param node Node for adding.
     */
    public void offer(Node<K, V> node) {
        if (size == 0) {
            // Put to the first.
            element = node;
            element.left = element;
            element.right = element;
        } else if (size == 1) {
            // Put after first and connect to each other.
            element.left = node;
            element.right = node;
            node.left = element;
            node.right = element;
        } else {
            // Put to the left after first element.
            element.left.right = node;
            node.left = element.left;
            node.right = element;
            element.left = node;
        }
        size++;
    }

    /**
     * Delete element by link to this element.
     * <br/>Time complexity - O(1)
     *
     * @param node Node for deleting.
     */
    public void poll(Node<K, V> node) {
        if (getSize() == 1) {
            // Remove the one element.
            element = null;
        } else if (node == element) {
            // Remove first element and connect 'element' to the next element.
            element.left.right = element.right;
            element.right.left = element.left;
            element = element.right;
        } else {
            // Just delete it.
            node.left.right = node.right;
            node.right.left = node.left;
        }
        size--;
    }
}

/**
 * Interface for Priority Queue.
 *
 * @param <K> Key
 * @param <V> Value
 */
interface IPriorityQueue<K, V> {
    void insert(Object item);

    Object findMin();

    Object extractMin();

    void decreaseKey(Object item, K newKey) throws Exception;

    void delete(Object item) throws Exception;

    void union(Object anotherQueue);
}

/**
 * Implementation of Priority Queue by using Fibonacci Heap.
 *
 * @param <K> Key with supporting compareTo() method.
 * @param <V> Value
 */
class FibHeap<K extends Comparable<? super K>, V> implements IPriorityQueue<K, V> {
    // Link to the minimum node.
    private Node<K, V> min;
    // Size of heap.
    private int n;
    // Root list of heap.
    private CircularQueue<K, V> rootList;
    // Minimum value of key for .delete() method.
    private final K minValue;

    /**
     * Creating the new heap.
     * <br/>Time complexity - O(1)
     *
     * @param minValue Minimum value of key for .delete() method.
     */
    public FibHeap(K minValue) {
        min = null;
        this.minValue = minValue;
        rootList = new CircularQueue<>();
        n = 0;
    }

    public boolean isEmpty() {
        return n == 0;
    }

    /**
     * Insert new node to the heap.
     * <br/>Time complexity - O(1).
     *
     * @param item
     */
    @Override
    public void insert(Object item) {
        Node<K, V> node = (Node<K, V>) item;
        if (min == null) {
            // Check for any minimum. Add first element.
            min = node;
        } else {
            // Just add and compare with current minimum.
            if (node.key.compareTo(min.key) < 0) {
                min = node;
            }
        }
        // Add new node to root list.
        rootList.offer(node);
        n++;
    }

    /**
     * Return minimum node of heap.
     * <br/>Time complexity - O(1).
     *
     * @return Node with minimum value.
     */
    @Override
    public Object findMin() {
        return min;
    }

    /**
     * Take minimum node from heap and delete it from heap.
     * <br/>Time complexity - O(log(n)).
     *
     * @return Node with minimum value.
     */
    @Override
    public Object extractMin() {
        // Check for nothing in heap.
        if (min == null) {
            return null;
        }
        // Node which will be extracted.
        Node<K, V> extracted = new Node<>(min);
        // Add all elements from root list to the array for clear walk.
        ArrayList<Node<K, V>> tempChildren = new ArrayList<>();
        Node<K, V> pointer = min.children.element;
        for (int i = 0; i < min.degree; i++) {
            tempChildren.add(pointer);
            pointer = pointer.right;
        }
        // Transfer all children of extracted node to the root list.
        for (int i = 0; i < min.degree; i++) {
            rootList.offer(tempChildren.get(i));
            tempChildren.get(i).parent = null;
        }
        // Delete extracted from root list.
        rootList.poll(min);
        if (min == min.right) {
            min = null;
        } else {
            min = min.right;
            // Consolidate root list.
            consolidate();
        }
        n--;
        return extracted;
    }

    /**
     * Sum subtrees if it needed.
     * <br/>Time complexity - O(log(n))
     */
    private void consolidate() {
        // Compute the maximum degree of every node.
        int D = (int) (Math.log(n) / 0.47 + 2);
        // Array for nodes which ordered by degree.
        ArrayList<Node<K, V>> degreeArray = new ArrayList<>(D);
        for (int i = 0; i < D; i++) {
            degreeArray.add(null);
        }
        Node<K, V> pointer = min;
        // Add all elements from root list to the array for clear walk.
        LinkedList<Node<K, V>> rootListTemp = new LinkedList<>();
        for (int i = 0; i < rootList.getSize(); i++) {
            rootListTemp.add(pointer);
            pointer = pointer.right;
        }
        // Go through all nodes from root list.
        for (Node<K, V> X : rootListTemp) {
            Node<K, V> x = X;
            int d = x.degree;
            // Sum nodes with the same degree.
            while (degreeArray.get(d) != null) {
                Node<K, V> y = degreeArray.get(d);
                if (x == y) {
                    break;
                }
                // Swap nodes if second one less than first.
                if (x.key.compareTo(y.key) > 0) {
                    Node<K, V> temp = x;
                    x = y;
                    y = temp;
                }
                // Remove second node from root list and add it to the children of first.
                rootList.poll(y);
                x.children.offer(y);
                y.parent = x;
                y.mark = false;
                x.degree++;
                degreeArray.set(d, null);
                d++;
            }
            // Set current node in the degree array.
            degreeArray.set(d, x);
        }
        min = null;
        // Compute new min and put all nodes from degree array to the root list.
        for (int i = 0; i < D; i++) {
            if (degreeArray.get(i) != null) {
                if (min == null) {
                    rootList = new CircularQueue<>();
                    min = degreeArray.get(i);
                    rootList.offer(min);
                } else {
                    rootList.offer(degreeArray.get(i));
                    if (degreeArray.get(i).key.compareTo(min.key) < 0) {
                        min = degreeArray.get(i);
                    }
                }
            }
        }
    }

    /**
     * Decrease the key for the following node.
     *
     * @param item   Node.
     * @param newKey Key for new node.
     * @throws Exception for new key which is greater than current key.
     */
    @Override
    public void decreaseKey(Object item, K newKey) throws Exception {
        Node<K, V> node = (Node<K, V>) item;
        if (newKey.compareTo(node.key) > 0) {
            throw new Exception("New key is greater than current key");
        }
        node.key = newKey;
        Node<K, V> parent = node.parent;
        if (parent != null && node.key.compareTo(parent.key) < 0) {
            cut(node, parent);
            cascadingCut(parent);
        }
        if (node.key.compareTo(min.key) < 0) {
            min = node;
        }
    }

    /**
     * @param node1
     * @param node2
     */
    private void cut(Node<K, V> node1, Node<K, V> node2) {
        node2.children.poll(node1);
        node2.degree--;
        rootList.offer(node1);
        node1.parent = null;
        node1.mark = false;
    }

    /**
     * @param node
     */
    private void cascadingCut(Node<K, V> node) {
        Node<K, V> parent = node.parent;
        if (parent != null) {
            if (!node.mark) {
                node.mark = true;
            } else {
                cut(node, parent);
                cascadingCut(parent);
            }
        }
    }

    /**
     * Delete node.
     * <br/>Time complexity - O()
     *
     * @param item Node for deleting.
     * @throws Exception
     */
    @Override
    public void delete(Object item) throws Exception {
        Node<K, V> node = (Node<K, V>) item;
        decreaseKey(node, minValue);
        extractMin();
    }

    /**
     * Insert all elements in the heap from another heap.
     * <br/>Time complexity - O(n)
     *
     * @param anotherQueue Another heap.
     */
    @Override
    public void union(Object anotherQueue) {
        FibHeap<K, V> second = (FibHeap<K, V>) anotherQueue;
        // Link to the root list element from another heap.
        Node<K, V> pointer = second.rootList.element;
        for (int i = 0; i < second.rootList.getSize(); i++) {
            Node<K, V> temp = pointer;
            pointer = pointer.right;
            // Add all elements from root list of another heap to the current root list heap.
            rootList.offer(temp);
        }
        // Consolidate root list.
        consolidate();
    }
}