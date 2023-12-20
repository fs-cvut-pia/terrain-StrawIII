#include "Letadlo.h"
#include <algorithm>
#include <queue>
#include <set>
#include <map>
#include <cmath>

Letadlo::Letadlo(TerrainMap &m, const Point &startIn, const Point &finishIn)
    : Path(m, "Latadlo", startIn, finishIn) {}

bool Letadlo::find()
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

std::vector<std::pair<Point, double>> Letadlo::findNeighbor(const Point &current)
{
    std::vector<std::pair<Point, double>> neighbors;
    for (int j = -1; j < 2; ++j)
    {
        for (int i = -1; i < 2; ++i)
        {
            auto neighbor = Point(current.x + i, current.y + j);
            double cost = (j == 0 || i == 0) ? 1.0 : std::sqrt(2.0); // If not diagonal, cost is 1, otherwise sqrt(2)
            if (isValidMove(neighbor))
            {
                neighbors.push_back(std::make_pair(neighbor, cost));
            }
        }
    }
    return neighbors;
}

bool Letadlo::isValidMove(const Point &referencePoint)
{
    return map.validCoords(referencePoint);
}

void Letadlo::reconstructPath(const std::map<Point, Point> &predecessor)
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
