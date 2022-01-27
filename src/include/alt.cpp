
#include "alt.h"

static Node* extractNode(std::vector<Node*>& set)
{
    int ind = 0;
    int minVal = INF;

    for (int i = 0; i < set.size(); i++)
    {
        if (set[i]->f < minVal)
        {
            ind = i;
            minVal = set[i]->f;
        }
    }

    Node* node = set[ind];
    set.erase(set.begin() + ind);

    return node;
}

static Node* extractNode(std::vector<Node*>& set, int numLand)
{
    int ind = 0;
    int minVal = INF;

    for (int i = 0; i < set.size(); i++)
    {
        if (set[i]->land[numLand] < minVal)
        {
            ind = i;
            minVal = set[i]->land[numLand];
        }
    }

    Node* node = set[ind];
    set.erase(set.begin() + ind);

    return node;
}

static bool checkNode(const std::vector<Node*>& set, const Node* current)
{
    for (int i = 0; i < set.size(); i++)
    {
        if (current->x == set[i]->x && current->y == set[i]->y)
            return true;
    }

    return false;
}

static int estimateDist(const Node& current, const Node& goal)
{
    int result = 0;

    for (int i = 0; i < 4; i++)
    {
        int dist = abs(current.land[i] - goal.land[i]);

        if (dist > result)
            result = dist;
    }

    return result;
}

static std::vector<Node*> getNeighbours(std::vector<Node>& graph, int rows, int cols, const Node* current)
{
    std::vector<Node*> neighbours;

    int indNeigh[4][2] = { {current->y + 1, current->x},
                           {current->y - 1, current->x},
                           {current->y, current->x + 1},
                           {current->y, current->x - 1} };

    for (int i = 0; i < 4; i++)
    {
        if (indNeigh[i][0] >= 0 && indNeigh[i][0] < rows)
        {
            if (indNeigh[i][1] >= 0 && indNeigh[i][1] < cols)
            {
                int ind = indNeigh[i][0] * rows + indNeigh[i][1];
                
                if (graph[ind].pass == 1)
                    neighbours.push_back(&graph[ind]);
            }
        }
    }

    return neighbours;
}


static void preprocess(std::vector<Node>& graph, int rows, int cols)
{
    int indLand[4] = { 0, 
                       cols - 1,
                       (rows - 1) * rows, 
                       (rows - 1) * rows + (cols - 1) };

    for (int i = 0; i < 4; i++)
    {
        int num = i;
        int ind = indLand[i];
        std::vector<Node*> next;
        std::vector<Node*> visited;

        for (int j = 0; j < graph.size(); j++)
            graph[j].land[num] = (j == ind) ? 0 : INF;

        next.push_back(&graph[ind]);

        while (!next.empty())
        {
            Node* current = extractNode(next, num);
            visited.push_back(current);

            std::vector<Node*> neighbours = getNeighbours(graph, rows, cols, current);

            for (int j = 0; j < neighbours.size(); j++)
            {
                if (!checkNode(visited, neighbours[j]))
                {
                    int tempDist = current->land[num] + 1;

                    if (tempDist < neighbours[j]->land[num])
                        neighbours[j]->land[num] = tempDist;

                    next.push_back(neighbours[j]);
                }
            }
        }
    }
}

static void search(std::vector<Node>& graph, int rows, int cols, int start, int goal)
{
    std::vector<Node*> next;
    std::vector<Node*> visited;

    for (int i = 0; i < graph.size(); i++)
    {
        if (graph[i].pass == 1)
        {
            if (i == start)
            {
                graph[i].g = 0;
                graph[i].h = estimateDist(graph[i], graph[goal]);
                graph[i].f = graph[i].g + graph[i].h;
            }
            else
            {
                graph[i].g = INF;
                graph[i].h = estimateDist(graph[i], graph[goal]);
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

    next.push_back(&graph[start]);

    while (!next.empty())
    {
        Node* current = extractNode(next);

        if (current->x == graph[goal].x && current->y == graph[goal].y)
            break;

        visited.push_back(current);

        std::vector<Node*> neighbours = getNeighbours(graph, rows, cols, current);

        for (int j = 0; j < neighbours.size(); j++)
        {
            if (!checkNode(visited, neighbours[j]))
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
        std::cout << "Path is not found!" << std::endl;
        return;
    }

    std::vector<const Node*> path;
    path.push_back(current);

    while (current->parent != -1)
    {
        current = &graph[current->parent];
        path.push_back(current);
    }

    std::cout << "Found path (row;column): " << std::endl;

    for (int i = path.size() - 1; i >= 0; i--)
    {
        std::cout << "(" << path[i]->y << ";" << path[i]->x << ")";
        if (i != 0)
            std::cout << ",";
    } 
}


void alt(const int graph[][9], int rows, int cols, int start, int goal)
{
    if (start == goal)
    {
        std::cout << "Start node is equal to goal node!" << std::endl;
        return;
    }

    std::vector<Node> cvtGraph;

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            Node node;
            node.x = j;
            node.y = i;
            node.pass = graph[i][j];
            node.parent = -1;

            cvtGraph.push_back(node);
        }
    }

    preprocess(cvtGraph, rows, cols);
    search(cvtGraph, rows, cols, start, goal);
    print(cvtGraph, goal);
}
