
#ifndef ALT_HPP
#define ALT_HPP

#include <iostream>
#include <vector>

const int INF = 10000;

struct Node
{
    int x;              // Координаты вершины
    int y;
    int f;              // Оценочная функция -> f = g + h
    int g;              // Расстояние от начальной вершины
    int h;              // Эвристическая функция
    int pass;           // Возможность прохода через вершину -> 1 = да, 0 = нет
    int parent;         // Вершина, из которой произошел переход
    int landmark[4];    // Расстояния до ориентиров (количество ориентиров = 4)
};

void alt(const int* graph, int rows, int cols, int start, int goal);

#endif // !ALT_HPP
