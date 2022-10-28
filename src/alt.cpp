
#include "alt.hpp"

static Node* extract_node(std::vector<Node*>& set)
{
    int min = INF;
    int i = -1;

    for (int j = 0; j < set.size(); ++j)
    {
        if (set[j]->f < min)
        {
            min = set[j]->f;
            i = j;
        }
    }

    Node* node = set[i];
    set.erase(set.begin() + i);

    return node;
}

static Node* extract_node(std::vector<Node*>& set, int numLand)
{
    int min = INF;
    int i = -1;

    for (int j = 0; j < set.size(); ++j)
    {
        if (set[j]->landmark[numLand] < min)
        {
            min = set[j]->landmark[numLand];
            i = j;
        }
    }

    Node *node = set[i];
    set.erase(set.begin() + i);

    return node;
}

static bool is_visited(const std::vector<Node*>& set, const Node* node)
{
    for (int i = 0; i < set.size(); ++i)
    {
        if (node == set[i])
            return true;
    }

    return false;
}

static int estimate_distance(const Node& node, const Node& goal)
{
    int result = 0;

    for (int i = 0; i < 4; ++i)
    {
        int dist = abs(node.landmark[i] - goal.landmark[i]);

        if (dist > result)
            result = dist;
    }

    return result;
}

static std::vector<Node*> get_neighbours(std::vector<Node>& graph, int rows, int cols, const Node* node)
{
    std::vector<Node*> neighbours;

    int neighbour[4][2] = { {node->y + 1, node->x},
                            {node->y - 1, node->x},
                            {node->y, node->x + 1},
                            {node->y, node->x - 1} };

    for (int i = 0; i < 4; ++i)
    {
        if (neighbour[i][0] >= 0 && neighbour[i][0] < rows)
        {
            if (neighbour[i][1] >= 0 && neighbour[i][1] < cols)
            {
                int j = neighbour[i][0] * rows + neighbour[i][1];

                if (graph[j].pass == 1)
                    neighbours.push_back(&graph[j]);
            }
        }
    }

    return neighbours;
}

static void preprocess(std::vector<Node>& graph, int rows, int cols)
{
    int landmark[4] = { 0, cols - 1, 
                        (rows - 1) * rows, (rows - 1) * rows + (cols - 1) };

    for (int i = 0; i < 4; ++i)
    {
        int numLand = i;
        int indLand = landmark[i];

        for (int j = 0; j < graph.size(); ++j)
            graph[j].landmark[numLand] = (j == indLand) ? 0 : INF;

        std::vector<Node *> next;
        std::vector<Node *> visited;

        next.push_back(&graph[indLand]);

        while (!next.empty())
        {
            Node *current = extract_node(next, numLand);
            visited.push_back(current);

            std::vector<Node*> neighbours = get_neighbours(graph, rows, cols, current);

            for (int j = 0; j < neighbours.size(); ++j)
            {
                if (!is_visited(visited, neighbours[j]))
                {
                    int tempDist = current->landmark[numLand] + 1;

                    if (tempDist < neighbours[j]->landmark[numLand])
                        neighbours[j]->landmark[numLand] = tempDist;

                    next.push_back(neighbours[j]);
                }
            }
        }
    }
}

static void search(std::vector<Node>& graph, int rows, int cols, int start, int goal)
{
    for (int i = 0; i < graph.size(); ++i)
    {
        if (graph[i].pass == 1)
        {
            if (i == start)
            {
                graph[i].g = 0;
                graph[i].h = estimate_distance(graph[i], graph[goal]);
                graph[i].f = graph[i].g + graph[i].h;
            }
            else
            {
                graph[i].g = INF;
                graph[i].h = estimate_distance(graph[i], graph[goal]);
                graph[i].f = INF;
            }
        }
        else
        {
            graph[i].g = INF;
            graph[i].h = INF;
            graph[i].f = INF;
        }
    }

    std::vector<Node *> next;
    std::vector<Node *> visited;

    next.push_back(&graph[start]);

    while (!next.empty())
    {
        Node* current = extract_node(next);
        visited.push_back(current);

        if (current == &graph[goal])
            break;

        std::vector<Node*> neighbours = get_neighbours(graph, rows, cols, current);

        for (int j = 0; j < neighbours.size(); ++j)
        {
            if (!is_visited(visited, neighbours[j]))
            {
                int tempDist = current->g + 1;

                if (tempDist < neighbours[j]->g)
                {
                    neighbours[j]->g = tempDist;
                    neighbours[j]->f = neighbours[j]->g + neighbours[j]->h;
                    neighbours[j]->parent = current->y * rows + current->x;
                }

                next.push_back(neighbours[j]);
            }
        }
    }
}

static void print(const std::vector<Node>& graph, int goal)
{
    const Node* current = &graph[goal];

    if (current->parent == -1)
    {
        std::cout << "Path is not found!";
        return;
    }

    std::vector<const Node*> path;

    while (current->parent != -1)
    {
        path.push_back(current);
        current = &graph[current->parent];
    }

    std::cout << "Found path (row;column): ";

    for (int i = path.size() - 1; i >= 0; --i)
    {
        std::cout << "(" << path[i]->y << ";" << path[i]->x << ")";
        if (i != 0)
            std::cout << ",";
    } 
}

void alt(const int* graph, int rows, int cols, int start, int goal)
{
    if (start == goal)
    {
        std::cout << "Start node is equal to goal node!";
        return;
    }

    std::vector<Node> cvtGraph;

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            Node node = {.x = j, .y = i, .pass = graph[i * rows + j], .parent = -1};
            cvtGraph.push_back(node);
        }
    }

    preprocess(cvtGraph, rows, cols);
    search(cvtGraph, rows, cols, start, goal);
    print(cvtGraph, goal);
}
