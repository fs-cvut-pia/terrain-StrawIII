#ifndef Lod_H_INCLUDED
#define Lod_H_INCLUDED
#include "Path.h"
#include <map>

class Lod : public Path
{
public:
    Lod(TerrainMap &m, const Point &startIn, const Point &finishIn);
    bool find() override;

private:
    bool isValidMove(const Point &referencePoint);
    std::vector<std::pair<Point, double>> findNeighbor(const Point &current);
    void reconstructPath(const std::map<Point, Point> &predecessor);
};

#endif // Lod_H_INCLUDED
