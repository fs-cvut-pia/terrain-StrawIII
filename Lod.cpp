#include "Lod.h"
#include <iostream>
#include <algorithm>
#include <queue>
#include <set>
#include <map>
#include <cmath>

Lodka::Lodka(TerrainMap &m, const Point &startIn, const Point &finishIn)
    : Path(m, "Lodka", startIn, finishIn) {}

bool Lodka::find()
{
    std::queue<Point> queue;
    std::set<Point> visited;
    std::map<Point, Point> predecessor;
    std::map<Point, double> cost;
    Point current;
    queue.push(start);
    visited.insert(start);
    cost[start] = 0;

    while (!queue.empty())
    {
        current = queue.front();
        queue.pop();

        if (current == finish)
        {
            reconstructPath(predecessor);
            return true;
        }

        for (const auto &neighbor : findNeighbor(current))
        {
            double newCost = cost[current] + neighbor.second;
            if (!visited.count(neighbor.first) || newCost < cost[neighbor.first])
            {
                cost[neighbor.first] = newCost;
                queue.push(neighbor.first);
                visited.insert(neighbor.first);
                predecessor[neighbor.first] = current;
            }
        }
    }
    return false;
}

std::vector<std::pair<Point, double>> Lodka::findNeighbor(const Point &current)
{
    Point neighbor;
    std::vector<std::pair<Point, double>> neighbors;

    for (int j = -1; j < 2; ++j)
    {
        for (int i = -1; i < 2; ++i)
        {
            neighbor = Point(current.x + i, current.y + j);
            double cost = (j == 0 || i == 0) ? 1.0 : std::sqrt(2.0);

            if (isValidMove(neighbor))
            {
                neighbors.push_back(std::make_pair(neighbor, cost));
            }
        }
    }
    return neighbors;
}

bool Lodka::isValidMove(const Point &referencePoint)
{
    return (map.validCoords(referencePoint) && (map.alt(referencePoint) < 0 || referencePoint == finish)) ? true : false;
}

void Lodka::reconstructPath(const std::map<Point, Point> &predecessor)
{
    Point current = finish;

    while (current != start)
    {
        path.push_back(current);
        current = predecessor.at(current);
    }

    path.push_back(current);
    std::reverse(path.begin(), path.end());
}
