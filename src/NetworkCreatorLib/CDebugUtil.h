#pragma once
#include <string>
#include "Boost3DPointHash.h"

class CDebugUtil
{
public:
    // ポリゴンのshapefile出力
    static bool OutputPolygonsToShp(
        const Boost3DHashMultiPolygon &polygons,
        std::string strShpPath,
        const bool isUseZ = false,
        const int nEpsgCode = 6668,
        const std::string strEncoding = "CP932");

    // ポイントのshapefile出力
    static bool OutputPointsToShp(
        const Boost3DMultiPointHashs &points,
        std::string strShpPath,
        const bool isUseZ = false,
        const int nEpsgCode = 6668,
        const std::string strEncoding = "CP932");

    // ポリラインのshapefile出力
    static bool OutputPolylinesToShp(
        const Boost3DHashMultiLines &polylines,
        std::string strShpPath,
        std::vector<double> widthList = std::vector<double>(),
        const bool isUseZ = false,
        const int nEpsgCode = 6668,
        const std::string strEncoding = "CP932");
};
