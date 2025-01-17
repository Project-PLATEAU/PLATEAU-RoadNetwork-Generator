#pragma once

#include "boost/geometry.hpp"

namespace bg = boost::geometry;

typedef bg::model::d2::point_xy<double> BoostPoint;                     // boostの2D座標型
typedef bg::model::d3::point_xyz<double> Boost3DPoint;                  // boostの3D座標型
typedef bg::model::linestring<BoostPoint> BoostPolyline;                // boostの2Dポリライン
typedef bg::model::multi_point<BoostPoint> BoostMultiPoints;            // 3D複数点型
typedef bg::model::multi_point<Boost3DPoint> Boost3DMultiPoints;        // 3D複数点型
typedef bg::model::multi_linestring<BoostPolyline> BoostMultiLines;     // 複数ライン型
typedef bg::model::box<BoostPoint> BoostBox;                            // 矩形型
typedef bg::model::ring<BoostPoint, false, true> BoostRing;             // 2Dリング型
typedef bg::model::ring<Boost3DPoint, false, true> Boost3DRing;         // 3Dリング型
typedef bg::model::polygon<BoostPoint, false, true> BoostPolygon;       // boostの2Dポリゴン
typedef bg::model::polygon<Boost3DPoint, false, true> Boost3DPolygon;   // boostの3Dポリゴン
typedef bg::model::multi_polygon<BoostPolygon> BoostMultiPolygon;       // boostの複数2Dポリゴン
typedef bg::model::multi_polygon<Boost3DPolygon> Boost3DMultiPolygon;   // boostの複数3Dポリゴン
// typedef bg::model::polygon<point, clock_wise, closed> polygon;
// clocke_wise = true(表:時計回り), false(表:反時計回り)
// clocke_wise = false(表:反時計回り)
// closed = true(終点に始点と同一点が挿入してポリゴンを閉じる必要がある)
// closed = false(終点に始点と同一点が挿入しなくても良い)
