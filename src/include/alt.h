
#ifndef ALT_H_
#define ALT_H_

#include <iostream>
#include <vector>

const int INF = 10000;

struct Node
{
    int x;          // ���������� �������
    int y;
    int f;          // ��������� ������� -> f = g + h
    int g;          // ���������� �� ��������� �������
    int h;          // ������������� �������
    int pass;       // ����������� ������� ����� ������� -> 1 = ��, 0 = ���
    int parent;     // �������, �� ������� ��������� �������
    int land[4];    // ���������� �� ���������� (���������� ���������� = 4)
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
