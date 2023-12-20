#ifndef LODKA_H_INCLUDED
#define LODKA_H_INCLUDED
#include "Path.h"
#include <map>

class Lodka : public Path
{
public:
    Lodka(TerrainMap &m, const Point &startIn, const Point &finishIn);
    bool find() override;

private:
    bool isValidMove(const Point &referencePoint);
    std::vector<std::pair<Point, double>> findNeighbor(const Point &current);
    void reconstructPath(const std::map<Point, Point> &predecessor);
};

#endif // LODKA_H_INCLUDED
