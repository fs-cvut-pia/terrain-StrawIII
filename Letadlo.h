#ifndef LETADLO_H_INCLUDED
#define LETADLO_H_INCLUDED
#include <map>
#include "Path.h"

class Letadlo : public Path
{
public:
    Letadlo(TerrainMap &m, const Point &startIn, const Point &finishIn);
    bool find() override;

private:
    bool isValidMove(const Point &referencePoint);
    std::vector<std::pair<Point, double>> findNeighbor(const Point &current);
    void reconstructPath(const std::map<Point, Point> &predecessor);
};
#endif // LETADLO_H_INCLUDED
