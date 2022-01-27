
#ifndef ALT_H_
#define ALT_H_

#include <iostream>
#include <vector>

const int INF = 10000;

struct Node
{
    int x;          // Координаты вершины
    int y;
    int f;          // Оценочная функция -> f = g + h
    int g;          // Расстояние от начальной вершины
    int h;          // Эвристическая функция
    int pass;       // Возможность прохода через вершину -> 1 = да, 0 = нет
    int parent;     // Вершина, из которой произошел переход
    int land[4];    // Расстояния до ориентиров (количество ориентиров = 4)
};

static Node* extractNode(std::vector<Node*>& set);
static Node* extractNode(std::vector<Node*>& set, int numLand);
static bool checkNode(const std::vector<Node*>& set, const Node* current);
static int estimateDist(const Node& current, const Node& goal);
static std::vector<Node*> getNeighbours(std::vector<Node>& graph, int rows, int cols, const Node* current);

static void preprocess(std::vector<Node>& graph, int rows, int cols);
static void search(std::vector<Node>& graph, int rows, int cols, int start, int goal);
static void print(const std::vector<Node>& graph, int goal);

void alt(const int graph[][9], int rows, int cols, int start, int goal);

#endif // !ALT_H_
