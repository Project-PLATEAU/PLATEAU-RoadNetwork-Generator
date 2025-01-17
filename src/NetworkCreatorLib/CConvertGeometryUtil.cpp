#include "CConvertGeometryUtil.h"
#include "CGeoUtil.h"
#include <string>

/*!
 * @brief CityGMLのポリゴンをBoostのポリゴンに変換
 * @param pPoly CityGMLのポリゴンのポインタ
 * @return Boostのポリゴン
*/
Boost3DHashPolygon CConvertGeometryUtil::ConvBoostPolygon(const std::shared_ptr<const citygml::Polygon> pPoly)
{
    return CConvertGeometryUtil::ConvBoostPolygon(pPoly, 0);
}

/*!
 * @brief CityGMLのポリゴンをBoostのポリゴンに変換(経緯度->平面直角座標系の変換付き)
 * @param pPoly     CityGMLのポリゴンのポインタ
 * @param nJPZone   平面直角座標系の系番号(1-19)
 * @return Boostのポリゴン
*/
Boost3DHashPolygon CConvertGeometryUtil::ConvBoostPolygon(
    const std::shared_ptr<const citygml::Polygon> pPoly,
    const int nJPZone)
{
    // 座標変換フラグ
    bool bConvCoordinate = (0 < nJPZone && 20 > nJPZone) ? true : false;

    bool bReverse = false;
    Boost3DHashPolygon poly;
    if (pPoly != nullptr)
    {
        BoostPolygon tmpPoly;   // 座標列反転判断用
        // 外輪郭
        if (pPoly->exteriorRing()->isExterior())
        {
            if (pPoly->exteriorRing()->getVertices().size() > 3)
            {
                for (const TVec3d &vertex : pPoly->exteriorRing()->getVertices())
                {
                    double dX = vertex.x;   // 緯度
                    double dY = vertex.y;   // 経度
                    double dZ = vertex.z;   // 高さ
                    if (bConvCoordinate)
                    {
                        CGeoUtil::LonLatToXY(vertex.y, vertex.x, nJPZone, dX, dY);
                    }

                    if (CEpsUtil::Zero(dX))
                        dX = 0;
                    if (CEpsUtil::Zero(dY))
                        dY = 0;
                    if (CEpsUtil::Zero(dZ))
                        dZ = 0;

                    poly.outer().push_back(Boost3DPoint(dX, dY, dZ));
                    tmpPoly.outer().push_back(BoostPoint(dX, dY));
                }
            }
        }

        if (bg::area(tmpPoly) < 0)
        {
            bReverse = true;    // 外輪郭の面積が負値の場合は座標列を反転する
            std::reverse(poly.outer().begin(), poly.outer().end());
        }

        // 穴
        auto interiorRings = pPoly->interiorRings();
        for (size_t interiorIdx = 0; interiorIdx < interiorRings.size(); interiorIdx++)
        {
            if (interiorRings.at(interiorIdx)->getVertices().size() > 3)
            {
                poly.inners().push_back(Boost3DHashPolygon::ring_type());
                for (const TVec3d &vertex : interiorRings.at(interiorIdx)->getVertices())
                {
                    double dX = vertex.x;   // 緯度
                    double dY = vertex.y;   // 経度
                    double dZ = vertex.z;   // 高さ
                    if (bConvCoordinate)
                    {
                        CGeoUtil::LonLatToXY(vertex.y, vertex.x, nJPZone, dX, dY);
                    }
                    if (CEpsUtil::Zero(dX))
                        dX = 0;
                    if (CEpsUtil::Zero(dY))
                        dY = 0;
                    if (CEpsUtil::Zero(dZ))
                        dZ = 0;

                    poly.inners().back().push_back(Boost3DPoint(dX, dY, dZ));
                }
                if (bReverse)
                    std::reverse(poly.inners().back().begin(), poly.inners().back().end());
            }
        }
    }
    return poly;
}
