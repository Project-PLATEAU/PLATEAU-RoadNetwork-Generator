#include "CTranRoadDataUtil.h"
#include "CConvertGeometryUtil.h"
#include "CGDALUtil.h"
#include "CBoostGeoUtil.h"
#include "SettingData.h"

/*!
 * @brief 道路LOD1ポリゴンの取得
 * @param pRoad     CityGMLの道路データのポインタ
 * @param polygon   道路LOD1ポリゴン
 * @param nJPZone   平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CTranRoadDataUtil::GetLOD1(
    const citygml::CityObject *pRoad,
    Boost3DHashPolygon &polygon,
    const int nJPZone)
{
    bool bRet = false;
    polygon.clear();
    if (pRoad != nullptr)
    {
        for (uint32_t geoIdx = 0; geoIdx < pRoad->getGeometriesCount(); geoIdx++)
        {
            auto geo = pRoad->getGeometry(geoIdx);
            if (geo.getLOD() == static_cast<uint32_t>(LOD_TYPE::LOD1))
            {
                // LOD1のためポリゴン数は1を想定
                for (uint32_t polyIdx = 0; polyIdx < geo.getPolygonsCount(); polyIdx++)
                {
                    std::shared_ptr<const citygml::Polygon> pPoly = geo.getPolygon(polyIdx);
                    polygon = CConvertGeometryUtil::ConvBoostPolygon(pPoly, nJPZone);
                }
                break;
            }
        }
        if (!bg::is_empty(polygon))
            bRet = true;
    }
    return bRet;
}

/*!
 * @brief 道路構造データの取得
 * @param pRoad CityGMLの道路データのポインタ
 * @param attr  道路構造データ
 * @return      道路構造データの有無
 * @retval      true    有り
 * @retval      false   無し
*/
bool CTranRoadDataUtil::GetRoadStructureAttribute(
    const citygml::CityObject *pRoad,
    CUroRoadStructureAttribute &attr)
{
    bool bRet = false;

    if (pRoad != nullptr)
    {
        auto it = pRoad->getAttributes().find(KEY_TRAN_STRUCTURE_ATTR);
        if (it != pRoad->getAttributes().end())
        {
            bRet = true;
            for (auto val : it->second.asAttributeSet())
            {
                if (val.first == KEY_TRAN_STRUCTURE_ATTR_NUMBER_OF_LANES)
                {
                    attr.nNumberOfLanes = val.second.asInteger();
                }
                else if (val.first == KEY_TRAN_STRUCTURE_ATTR_SECTION_TYPE)
                {
                    attr.strSectionType = val.second.asString();
                }
                else if (val.first == KEY_TRAN_STRUCTURE_ATTR_WIDTH)
                {
                    attr.dWidth = val.second.asDouble();
                }
                else if (val.first == KEY_TRAN_STRUCTURE_ATTR_WIDTH_TYPE)
                {
                    attr.strWidthType = val.second.asString();
                }
            }
        }
    }

    return bRet;
}

/*!
 * @brief 道路品質情報のlodType(LOD3の場合の詳細度)を取得する
 * @param pRoad     CityGMLの道路データのポインタ
 * @param dLodType  LOD3の場合の詳細度
 * @return      データの有無
 * @retval      true    有り
 * @retval      false   無し
*/
bool CTranRoadDataUtil::GetTranDataQualityAttributeLodType(
    const citygml::CityObject *pRoad, double &dLodType)
{
    bool bRet = false;

    if (pRoad != nullptr)
    {
        auto it = pRoad->getAttributes().find(KEY_TRAN_DATA_QUALITY_ATTR);
        if (it != pRoad->getAttributes().end())
        {
            for (auto val : it->second.asAttributeSet())
            {
                if (val.first == KEY_TRAN_DATA_QUALITY_ATTR_LOD_TYPE)
                {
                    bRet = true;
                    std::string strVal = val.second.asString();
                    if (strVal.compare(KEY_TRAN_DATA_QUALITY_ATTR_LOD_TYPE_30) == 0)
                    {
                        dLodType = 3.0;
                    }
                    else if (strVal.compare(KEY_TRAN_DATA_QUALITY_ATTR_LOD_TYPE_31) == 0)
                    {
                        dLodType = 3.1;
                    }
                    else if (strVal.compare(KEY_TRAN_DATA_QUALITY_ATTR_LOD_TYPE_32) == 0)
                    {
                        dLodType = 3.2;
                    }
                    else if (strVal.compare(KEY_TRAN_DATA_QUALITY_ATTR_LOD_TYPE_33) == 0)
                    {
                        dLodType = 3.3;
                    }
                    else if (strVal.compare(KEY_TRAN_DATA_QUALITY_ATTR_LOD_TYPE_34) == 0)
                    {
                        dLodType = 3.4;
                    }
                    break;
                }
            }
        }
    }

    return bRet;
}


/*!
 * @brief CityObjectから道路データを取得
 * @param pRoad         CityGMLの道路データのポインタ
 * @param tranRoadData  道路LOD1ポリゴン
 * @param nJPZone       平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
*/
void CTranRoadDataUtil::GetTranRoadData(
    const citygml::CityObject *pRoad,
    CTranRoadData &tranRoadData,
    const int nJPZone,
    bool &bLod1Result,
    bool &bLod2Result,
    bool &bLod3Result)
{
    // id
    tranRoadData.m_strId = pRoad->getId();

    // LOD1の取得
    bLod1Result = getTranRoadDataLOD1(pRoad, tranRoadData, nJPZone);

    // LOD2, LOD3の取得
    getTranRoadDataLOD23(pRoad, tranRoadData, nJPZone, bLod2Result, bLod3Result);

    // 属性(CUroRoadStructureAttribute)の取得(tran:Roadに0又は1個存在)
    GetRoadStructureAttribute(pRoad, tranRoadData.m_roadStructureAttr);
}


/*!
 * @brief CityObjectから道路LOD1の必要なデータを取得
 * @param pRoad         CityGMLの道路データのポインタ
 * @param tranRoadData  道路LOD1ポリゴン
 * @param nJPZone       平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CTranRoadDataUtil::getTranRoadDataLOD1(
    const citygml::CityObject* pRoad,
    CTranRoadData& tranRoadData,
    const int nJPZone)
{
    bool bRet = false;

    auto& targetData = tranRoadData.m_lod1;

    Boost3DHashMultiPolygon lod1PolygonList;

    if (pRoad != nullptr)
    {
        for (uint32_t geoIdx = 0; geoIdx < pRoad->getGeometriesCount(); geoIdx++)
        {
            auto geo = pRoad->getGeometry(geoIdx);

            if (geo.getLOD() == static_cast<uint32_t>(LOD_TYPE::LOD1))
            {
                if (geo.getPolygonsCount() > 0)
                {
                    for (uint32_t lod1GeoIdx = 0; lod1GeoIdx < geo.getPolygonsCount(); lod1GeoIdx++)
                    {
                        std::shared_ptr<const citygml::Polygon> pPoly = geo.getPolygon(lod1GeoIdx);
                        auto polygon = CConvertGeometryUtil::ConvBoostPolygon(pPoly, nJPZone);
                        if (!bg::is_empty(polygon))
                        {
                            lod1PolygonList.push_back(polygon);
                        }
                    }
                }
            }
        }

        ///
        /// ポリゴンを融合してtranRoadDataに保存する
        ///
        Boost3DHashMultiPolygon dissolvedPolygons;
        lod1PolygonList.size() > 1 ? dissolvedPolygons = CGDALUtil::GetInstance()->Dissolve(lod1PolygonList) : dissolvedPolygons = lod1PolygonList;
        if (bg::is_empty(dissolvedPolygons) == false)
        {
            targetData.m_boostGeometry = dissolvedPolygons[0];

            bRet = true;
        }
    }

    return bRet;
}

/*!
 * @brief CityObjectから道路LOD2,3のデータを取得
 * @param pRoad         CityGMLの道路データのポインタ
 * @param tranRoadDataList  道路LOD2ポリゴンリスト
 * @param nJPZone       平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
 * @param bLod2Result   LOD2取得結果
 * @param bLod3Result   LOD3取得結果
*/
void CTranRoadDataUtil::getTranRoadDataLOD23(
    const citygml::CityObject* pRoad,
    CTranRoadData& tranRoadData,
    const int nJPZone,
    bool &bLod2Result,
    bool &bLod3Result)
{
    bLod2Result = false;
    bLod3Result = false;

    auto& targetLod2List = tranRoadData.m_lod2List;
    auto &targetLod3List = tranRoadData.m_lod3List;
    auto& targetLod3TriangularMeshList = tranRoadData.m_lod3TriangularMeshList;

    Boost3DHashMultiPolygon lod2RoadwayPolygonList;
    Boost3DHashMultiPolygon lod2LanePolygonList;
    Boost3DHashMultiPolygon lod2IntersectionPolygonList;
    Boost3DHashMultiPolygon lod2FootpathPolygonList;
    Boost3DHashMultiPolygon lod2IslandPolygonList;
    Boost3DHashMultiPolygon lod2PlantsPolygonList;

    Boost3DHashMultiPolygon lod3RoadwayPolygonList;
    Boost3DHashMultiPolygon lod3LanePolygonList;
    Boost3DHashMultiPolygon lod3IntersectionPolygonList;
    Boost3DHashMultiPolygon lod3FootpathPolygonList;
    Boost3DHashMultiPolygon lod3IslandPolygonList;
    Boost3DHashMultiPolygon lod3PlantsPolygonList;

    if (pRoad != nullptr)
    {
        // LOD3詳細度取得
        // LOD3の場合は、LOD3での詳細度が必ず設定されている仕様となっているため
        // データが無い場合はデータ不備
        double lodType = 3.0;
        CTranRoadDataUtil::GetTranDataQualityAttributeLodType(pRoad, lodType);

        for (uint32_t childIdx = 0; childIdx < pRoad->getChildCityObjectsCount(); childIdx++)
        {
            auto& childObj = pRoad->getChildCityObject(childIdx);
            std::string attrStr = childObj.getAttribute(KEY_TRAN_FUNCTION);
            const int nFunctionType = std::atoi(attrStr.c_str());

            for (uint32_t childGeoIdx = 0; childGeoIdx < childObj.getGeometriesCount(); childGeoIdx++)
            {
                auto geo = childObj.getGeometry(childGeoIdx);

                if (geo.getLOD() == static_cast<uint32_t>(LOD_TYPE::LOD2))
                {
                    for (uint32_t lod2GeoIdx = 0; lod2GeoIdx < geo.getPolygonsCount(); lod2GeoIdx++)
                    {
                        std::shared_ptr<const citygml::Polygon> pPoly = geo.getPolygon(lod2GeoIdx);
                        auto polygon = CConvertGeometryUtil::ConvBoostPolygon(pPoly, nJPZone);
                        if (!bg::is_empty(polygon))
                        {
                            switch (nFunctionType)
                            {
                            case (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY:
                                lod2RoadwayPolygonList.emplace_back(polygon);
                                break;
                            case (int)TRAFFIC_AREA_FUNCTION_TYPE::LANE:
                                lod2LanePolygonList.emplace_back(polygon);
                                break;
                            case (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION:
                                lod2IntersectionPolygonList.emplace_back(polygon);
                                break;
                            case (int)TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH:
                                lod2FootpathPolygonList.emplace_back(polygon);
                                break;
                            case (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND:
                                lod2IslandPolygonList.emplace_back(polygon);
                                break;
                            case (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::PLANTS:
                                lod2PlantsPolygonList.emplace_back(polygon);
                                break;
                            default:
                                break;
                            }

                        }
                    }
                }
                else if (geo.getLOD() == static_cast<uint32_t>(LOD_TYPE::LOD3))
                {
                    for (uint32_t lod3GeoIdx = 0; lod3GeoIdx < geo.getPolygonsCount(); lod3GeoIdx++)
                    {
                        std::shared_ptr<const citygml::Polygon> pPoly = geo.getPolygon(lod3GeoIdx);
                        auto polygon = CConvertGeometryUtil::ConvBoostPolygon(pPoly, nJPZone);
                        if (!bg::is_empty(polygon))
                        {
                            switch (nFunctionType)
                            {
                            case (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY:
                                lod3RoadwayPolygonList.emplace_back(polygon);
                                break;
                            case (int)TRAFFIC_AREA_FUNCTION_TYPE::LANE:
                                lod3LanePolygonList.emplace_back(polygon);
                                break;
                            case (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION:
                                lod3IntersectionPolygonList.emplace_back(polygon);
                                break;
                            case (int)TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH:
                                lod3FootpathPolygonList.emplace_back(polygon);
                                break;
                            case (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND:
                                lod3IslandPolygonList.emplace_back(polygon);
                                break;
                            case (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::PLANTS:
                                lod3PlantsPolygonList.emplace_back(polygon);
                                break;
                            default:
                                break;
                            }

                            // 高さ計測用に三角メッシュそのものも保存する
                            CTranRoadDataLod3 childTriMeshData;
                            childTriMeshData.m_fuctionType = nFunctionType;
                            childTriMeshData.m_dLod3Type = lodType;
                            childTriMeshData.m_boostGeometry = polygon;
                            targetLod3TriangularMeshList.emplace_back(childTriMeshData);
                        }
                    }
                }
            }
        }

        ///
        /// 各ポリゴンを融合してtranRoadDataに保存する
        ///

        // LOD2
        Boost3DHashMultiPolygon dissolvedPolygons;
        lod2RoadwayPolygonList.size() > 1 ? dissolvedPolygons = CGDALUtil::GetInstance()->Dissolve(lod2RoadwayPolygonList) : dissolvedPolygons = lod2RoadwayPolygonList;
        if (bg::is_empty(dissolvedPolygons) == false)
        {
            for (const auto& polygon : dissolvedPolygons)
            {
                CTranRoadDataLod2 childData;

                childData.m_fuctionType = (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY;
                childData.m_boostGeometry = polygon;

                targetLod2List.emplace_back(childData);
            }

            bLod2Result = true;
        }

        dissolvedPolygons.clear();
        lod2LanePolygonList.size() > 1 ? dissolvedPolygons = CGDALUtil::GetInstance()->Dissolve(lod2LanePolygonList) : dissolvedPolygons = lod2LanePolygonList;
        if (bg::is_empty(dissolvedPolygons) == false)
        {
            for (const auto& polygon : dissolvedPolygons)
            {
                CTranRoadDataLod2 childData;

                childData.m_fuctionType = (int)TRAFFIC_AREA_FUNCTION_TYPE::LANE;
                childData.m_boostGeometry = polygon;

                targetLod2List.emplace_back(childData);
            }

            bLod2Result = true;
        }

        dissolvedPolygons.clear();
        lod2IntersectionPolygonList.size() > 1 ? dissolvedPolygons = CGDALUtil::GetInstance()->Dissolve(lod2IntersectionPolygonList) : dissolvedPolygons = lod2IntersectionPolygonList;
        if (bg::is_empty(dissolvedPolygons) == false)
        {
            for (const auto& polygon : dissolvedPolygons)
            {
                CTranRoadDataLod2 childData;

                childData.m_fuctionType = (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION;
                childData.m_boostGeometry = polygon;

                targetLod2List.emplace_back(childData);
            }

            bLod2Result = true;
        }

        dissolvedPolygons.clear();
        lod2FootpathPolygonList.size() > 1 ? dissolvedPolygons = CGDALUtil::GetInstance()->Dissolve(lod2FootpathPolygonList) : dissolvedPolygons = lod2FootpathPolygonList;
        if (bg::is_empty(dissolvedPolygons) == false)
        {
            for (const auto& polygon : dissolvedPolygons)
            {
                CTranRoadDataLod2 childData;

                childData.m_fuctionType = (int)TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH;
                childData.m_boostGeometry = polygon;

                targetLod2List.emplace_back(childData);
            }

            bLod2Result = true;
        }

        dissolvedPolygons.clear();
        lod2IslandPolygonList.size() > 1 ? dissolvedPolygons = CGDALUtil::GetInstance()->Dissolve(lod2IslandPolygonList) : dissolvedPolygons = lod2IslandPolygonList;
        if (bg::is_empty(dissolvedPolygons) == false)
        {
            for (const auto& polygon : dissolvedPolygons)
            {
                CTranRoadDataLod2 childData;

                childData.m_fuctionType = (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND;
                childData.m_boostGeometry = polygon;

                targetLod2List.emplace_back(childData);
            }

            bLod2Result = true;
        }

        dissolvedPolygons.clear();
        lod2PlantsPolygonList.size() > 1 ? dissolvedPolygons = CGDALUtil::GetInstance()->Dissolve(lod2PlantsPolygonList) : dissolvedPolygons = lod2PlantsPolygonList;
        if (bg::is_empty(dissolvedPolygons) == false)
        {
            for (const auto& polygon : dissolvedPolygons)
            {
                CTranRoadDataLod2 childData;

                childData.m_fuctionType = (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::PLANTS;
                childData.m_boostGeometry = polygon;

                targetLod2List.emplace_back(childData);
            }

            bLod2Result = true;
        }

        // LOD3
        dissolvedPolygons.clear();
        lod3RoadwayPolygonList.size() > 1 ? dissolvedPolygons = CGDALUtil::GetInstance()->Dissolve(lod3RoadwayPolygonList) : dissolvedPolygons = lod3RoadwayPolygonList;
        if (bg::is_empty(dissolvedPolygons) == false)
        {
            for (const auto& polygon : dissolvedPolygons)
            {
                CTranRoadDataLod3 childData;

                childData.m_fuctionType = (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY;
                childData.m_dLod3Type = lodType;
                childData.m_boostGeometry = polygon;

                targetLod3List.emplace_back(childData);
            }

            bLod3Result = true;
        }

        dissolvedPolygons.clear();
        lod3LanePolygonList.size() > 1 ? dissolvedPolygons = CGDALUtil::GetInstance()->Dissolve(lod3LanePolygonList) : dissolvedPolygons = lod3LanePolygonList;
        if (bg::is_empty(dissolvedPolygons) == false)
        {
            for (const auto& polygon : dissolvedPolygons)
            {
                CTranRoadDataLod3 childData;

                childData.m_fuctionType = (int)TRAFFIC_AREA_FUNCTION_TYPE::LANE;
                childData.m_dLod3Type = lodType;
                childData.m_boostGeometry = polygon;

                targetLod3List.emplace_back(childData);
            }

            bLod3Result = true;
        }

        dissolvedPolygons.clear();
        lod3IntersectionPolygonList.size() > 1 ? dissolvedPolygons = CGDALUtil::GetInstance()->Dissolve(lod3IntersectionPolygonList) : dissolvedPolygons = lod3IntersectionPolygonList;
        if (bg::is_empty(dissolvedPolygons) == false)
        {
            for (const auto& polygon : dissolvedPolygons)
            {
                CTranRoadDataLod3 childData;

                childData.m_fuctionType = (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION;
                childData.m_dLod3Type = lodType;
                childData.m_boostGeometry = polygon;

                targetLod3List.emplace_back(childData);
            }

            bLod3Result = true;
        }

        dissolvedPolygons.clear();
        lod3FootpathPolygonList.size() > 1 ? dissolvedPolygons = CGDALUtil::GetInstance()->Dissolve(lod3FootpathPolygonList) : dissolvedPolygons = lod3FootpathPolygonList;
        if (bg::is_empty(dissolvedPolygons) == false)
        {
            for (const auto& polygon : dissolvedPolygons)
            {
                CTranRoadDataLod3 childData;

                childData.m_fuctionType = (int)TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH;
                childData.m_dLod3Type = lodType;
                childData.m_boostGeometry = polygon;

                targetLod3List.emplace_back(childData);
            }

            bLod3Result = true;
        }

        dissolvedPolygons.clear();
        lod3IslandPolygonList.size() > 1 ? dissolvedPolygons = CGDALUtil::GetInstance()->Dissolve(lod3IslandPolygonList) : dissolvedPolygons = lod3IslandPolygonList;
        if (bg::is_empty(dissolvedPolygons) == false)
        {
            for (const auto& polygon : dissolvedPolygons)
            {
                CTranRoadDataLod3 childData;

                childData.m_fuctionType = (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND;
                childData.m_dLod3Type = lodType;
                childData.m_boostGeometry = polygon;

                targetLod3List.emplace_back(childData);
            }

            bLod3Result = true;
        }

        dissolvedPolygons.clear();
        lod3PlantsPolygonList.size() > 1 ? dissolvedPolygons = CGDALUtil::GetInstance()->Dissolve(lod3PlantsPolygonList) : dissolvedPolygons = lod3PlantsPolygonList;
        if (bg::is_empty(dissolvedPolygons) == false)
        {
            for (const auto& polygon : dissolvedPolygons)
            {
                CTranRoadDataLod3 childData;

                childData.m_fuctionType = (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::PLANTS;
                childData.m_dLod3Type = lodType;
                childData.m_boostGeometry = polygon;

                targetLod3List.emplace_back(childData);
            }

            bLod3Result = true;
        }
    }

}

// CityObjectからtran:functionを取得
bool CTranRoadDataUtil::GetTranFunction(
    const citygml::CityObject *pRoad,
    CTranRoadData &tranRoadData)
{
    bool bRet = false;
    tranRoadData.m_strFunction = "";

    if (pRoad != nullptr)
    {
        auto it = pRoad->getAttributes().find(KEY_TRAN_FUNCTION);
        if (it != pRoad->getAttributes().end())
        {
            bRet = true;
            tranRoadData.m_strFunction = it->second.asString();
        }
    }
    return bRet;
}

// CityObjectからgml:nameを取得
bool CTranRoadDataUtil::GetGmlName(
    const citygml::CityObject *pRoad,
    CTranRoadData &tranRoadData)
{
    bool bRet = false;
    tranRoadData.m_strName = "";

    if (pRoad != nullptr)
    {
        auto it = pRoad->getAttributes().find(KEY_GML_NAME);
        if (it != pRoad->getAttributes().end())
        {
            bRet = true;
            tranRoadData.m_strName = it->second.asString();
        }
    }
    return bRet;
}

// 歩道部と植栽を融合したポリゴンを取得する(LOD3.2以上用)
Boost3DHashMultiPolygon CTranRoadDataUtil::GetFootpathAndPlantsPolygon(const CTranRoadData &tranRoadData)
{
    Boost3DHashMultiPolygon footpathPolygonList;    // 歩道部ポリゴン
    Boost3DHashMultiPolygon plantPolygonList;       // LOD3.2以上の植栽用
    Boost3DHashMultiPolygon ret;

    for (auto lod3 : tranRoadData.m_lod3List)
    {
        if (lod3.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH))
            footpathPolygonList.emplace_back(lod3.m_boostGeometry);   // 歩道部
        else if (lod3.m_fuctionType == static_cast<std::underlying_type<AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE>::type>(AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::PLANTS))
            plantPolygonList.emplace_back(lod3.m_boostGeometry);    // lod3.2以上の植栽
    }

    if (plantPolygonList.size() > 0 && footpathPolygonList.size() > 0)
    {
        // LOD3.2以上で植栽と歩道部がある場合は、歩道部と植栽を融合したポリゴンを使用する
        Boost3DHashMultiPolygon tmp;
        for (const auto &plantPolygon : plantPolygonList)
        {
            for (const auto &footpathPolygon : footpathPolygonList)
            {
                if (!CBoostGeoUtil::Disjoint(footpathPolygon, plantPolygon))
                {
                    tmp.emplace_back(plantPolygon);
                    break;
                }
            }
        }
        if (tmp.size() > 0)
        {
            // 歩道部と植栽を融合したポリゴンを作成する
            tmp.insert(tmp.end(), footpathPolygonList.begin(), footpathPolygonList.end());
            ret = CGDALUtil::GetInstance()->Dissolve(tmp);
        }
        else
        {
            // 歩道部と衝突する植栽がない場合
            ret = footpathPolygonList;
        }
    }
    else
    {
        // 植栽がない場合
        ret = footpathPolygonList;
    }
    return ret;
}

// 入力歩道中心線に衝突する歩道部ポリゴンの探索
bool CTranRoadDataUtil::SearchFootpathPolygon(
    const CTranRoadData &tranRoadData,
    const Boost3DHashPolyline &centerLine,
    Boost3DHashPolygon &polygon)
{
    bool bRet = false;
    Boost3DHashMultiPolygon footPathPolygons;
    switch (CInputSettingData::GetInstance()->lodType)
    {
    case CInputSettingData::LODType::LOD2:
        for (auto lod2 : tranRoadData.m_lod2List)
        {
            // 歩道部のみ
            if (lod2.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(
                TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH))
            {
                footPathPolygons.emplace_back(lod2.m_boostGeometry);
            }
        }
        break;
    case CInputSettingData::LODType::LOD3:
        footPathPolygons = CTranRoadDataUtil::GetFootpathAndPlantsPolygon(tranRoadData);
        break;
    default:
        break;
    }

    // 中心線に対応する歩道ポリゴンを取得
    for (const auto &tmp : footPathPolygons)
    {
        if (!CBoostGeoUtil::Disjoint(tmp, centerLine))
        {
            polygon = tmp;
            bRet = true;
            break;
        }
    }

    return bRet;
}
