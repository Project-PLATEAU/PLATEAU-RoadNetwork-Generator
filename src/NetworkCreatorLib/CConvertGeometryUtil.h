#pragma once
#include "Boost3DPointHash.h"
#include "CityGMLCommon.h"
#include "CUtil.h"
#include "CommonDef.h"
#include "gdal/ogrsf_frmts.h"

/*!
 * @brief CityGML/GDALの幾何情報をBoostの幾何情報に変換するユーティリティクラス
*/
class CConvertGeometryUtil
{
public:
    /*!
     * @brief CityGMLのポリゴンをBoostのポリゴンに変換
     * @param pPoly CityGMLのポリゴンのポインタ
     * @return Boostのポリゴン
    */
    static Boost3DHashPolygon ConvBoostPolygon(const std::shared_ptr<const citygml::Polygon> pPoly);

    /*!
     * @brief CityGMLのポリゴンをBoostのポリゴンに変換(経緯度->平面直角座標系の変換付き)
     * @param pPoly     CityGMLのポリゴンのポインタ
     * @param nJPZone   平面直角座標系の系番号(1-19)
     * @return Boostのポリゴン
    */
    static Boost3DHashPolygon ConvBoostPolygon(
        const std::shared_ptr<const citygml::Polygon> pPoly,
        const int nJPZone);

    /*!
     * @brief Boost3DPointHash->OGRPointへの変換
     * @param pt        Boost3DPointHash
     * @param isUseZ    z座標の使用可否
     * @param isRound   丸め座標の使用可否
     * @param nDigit    丸め座標使用時の小数点以下の桁数
     * @return OGRPoint
    */
    static OGRPoint ConvOgrPoint(
        const Boost3DPointHash &pt,
        const bool isUseZ = true,
        const bool isRound = false,
        const int nDigit = ncl_common_def::POINT_SIGNIFICANT_DIGITS)
    {
        double dX = (isRound) ? CUtil::RoundN(pt.x(), nDigit) : pt.x();
        double dY = (isRound) ? CUtil::RoundN(pt.y(), nDigit) : pt.y();
        double dZ = (isRound) ? CUtil::RoundN(pt.z(), nDigit) : pt.z();
        if (isUseZ)
        {
            return OGRPoint(dX, dY, dZ);
        }
        else
        {
            return OGRPoint(dX, dY);
        }
    }

    /*!
     * @brief OGRPoint->Boost3DPointHashへの変換
     * @param pt OGRPoint
     * @return Boost3DPointHash
    */
    static Boost3DPointHash ConvBoostPoint(const OGRPoint &pt)
    {
        double dZ = (pt.CoordinateDimension() > 2) ? pt.getZ() : 0;
        return Boost3DPointHash(pt.getX(), pt.getY(), dZ);
    }

    /*!
     * @brief Boost3DHashPolygon->OGRPolygonへの変換
     * @param poly      Boost3DHashPolygon
     * @param isUseZ    z座標の使用可否
     * @param isRound   丸め座標の使用可否
     * @param nDigit    丸め座標使用時の小数点以下の桁数
     * @return     OGRPolygon
    */
    static OGRPolygon ConvOgrPolygon(
        const Boost3DHashPolygon &poly,
        const bool isUseZ = true,
        const bool isRound = false,
        const int nDigit = ncl_common_def::POINT_SIGNIFICANT_DIGITS)
    {
        OGRPolygon dstPoly;
        OGRLinearRing ogrOuter;
        for (const auto &pt : poly.outer())
        {
            OGRPoint ogrPt = ConvOgrPoint(pt, isUseZ, isRound, nDigit);
            ogrOuter.addPoint(&ogrPt);
        }
        dstPoly.addRing(&ogrOuter);

        for (const auto &ring : poly.inners())
        {
            OGRLinearRing ogrInner;
            for (const auto &pt : ring)
            {
                OGRPoint ogrPt = ConvOgrPoint(pt, isUseZ, isRound, nDigit);
                ogrInner.addPoint(&ogrPt);
            }
            dstPoly.addRing(&ogrInner);
        }

        return dstPoly;
    }

    /*!
     * @brief Boost3DHashPolygon->OGRPolygonポインタへの変換
     * @param poly      Boost3DHashPolygon
     * @param isUseZ    z座標の使用可否
     * @param isRound   丸め座標の使用可否
     * @param nDigit    丸め座標使用時の小数点以下の桁数
     * @return          OGRPolygonポインタ(NULLの場合は失敗)
     * @note            戻り値は使用後にメモリ解放する必要あり
    */
    static OGRPolygon * ConvOgrPolygonPtr(
        const Boost3DHashPolygon &poly,
        const bool isUseZ = true,
        const bool isRound = false,
        const int nDigit = ncl_common_def::POINT_SIGNIFICANT_DIGITS)
    {
        OGRPolygon *pDstGeom = static_cast<OGRPolygon *>(OGRGeometryFactory::createGeometry(OGRwkbGeometryType::wkbPolygon));
        if (pDstGeom != nullptr)
        {
            OGRLinearRing ogrOuter;
            for (const auto &pt : poly.outer())
            {
                OGRPoint ogrPt = ConvOgrPoint(pt, isUseZ, isRound, nDigit);
                ogrOuter.addPoint(&ogrPt);
            }
            pDstGeom->addRing(&ogrOuter);

            for (const auto &ring : poly.inners())
            {
                OGRLinearRing ogrInner;
                for (const auto &pt : ring)
                {
                    OGRPoint ogrPt = ConvOgrPoint(pt, isUseZ, isRound, nDigit);
                    ogrInner.addPoint(&ogrPt);
                }
                pDstGeom->addRing(&ogrInner);
            }
        }
        return pDstGeom;
    }

    /*!
     * @brief OGRPolygon->Boost3DHashPolygonへの変換
     * @param poly OGRPolygon
     * @return Boost3DPolygon
    */
    static Boost3DHashPolygon ConvBoostPolygon(const OGRPolygon &poly)
    {
        bool bReverse = false;
        BoostPolygon tmpPoly;
        Boost3DHashPolygon dstPoly;
        for (const auto &pt : poly.getExteriorRing())
        {
            Boost3DPointHash boostPt(pt.getX(), pt.getY(), pt.getZ());
            dstPoly.outer().push_back(boostPt);
            tmpPoly.outer().push_back(BoostPoint(pt.getX(), pt.getY()));
        }

        if (bg::area(tmpPoly) < 0)
        {
            bReverse = true;
            std::reverse(dstPoly.outer().begin(), dstPoly.outer().end());
        }

        for (int i = 0; i < poly.getNumInteriorRings(); i++)
        {
            dstPoly.inners().push_back(Boost3DHashPolygon::ring_type());
            for (const auto &pt : poly.getInteriorRing(i))
            {
                Boost3DPointHash boostPt(pt.getX(), pt.getY(), pt.getZ());
                dstPoly.inners().back().push_back(boostPt);
            }
            if (bReverse)
            {
                std::reverse(dstPoly.inners().back().begin(), dstPoly.inners().back().end());
            }
        }
        //bg::unique(dstPoly);
        //bg::correct(dstPoly);
        return dstPoly;
    }

    /*!
     * @brief Boost3DHashPolyline->OGRLineStringへの変換
     * @param poly      Boost3DHashPolyline
     * @param isUseZ    z座標の使用可否
     * @param isRound   丸め座標の使用可否
     * @param nDigit    丸め座標使用時の小数点以下の桁数
s     * @return     OGRLineString
    */
    static OGRLineString ConvOgrLineString(
        const Boost3DHashPolyline &polyline,
        const bool isUseZ = true,
        const bool isRound = false,
        const int nDigit = ncl_common_def::POINT_SIGNIFICANT_DIGITS)
    {
        OGRLineString dstPolyline;
        for (const auto &pt : polyline)
        {
            OGRPoint ogrPt = ConvOgrPoint(pt, isUseZ, isRound, nDigit);
            dstPolyline.addPoint(&ogrPt);
        }
        return dstPolyline;
    }

    /*!
     * @brief OGRLineString->Boost3DHashPolylineへの変換
     * @param polyline OGRLineString
     * @return Boost3DHashPolyline
    */
    static Boost3DHashPolyline ConvBoostPolyline(const OGRLineString &polyline)
    {
        Boost3DHashPolyline dstPolyline;
        for (const auto &pt : polyline)
        {
            Boost3DPointHash boostPt(pt.getX(), pt.getY(), pt.getZ());
            dstPolyline.push_back(boostPt);
        }
        return dstPolyline;
    }
};

