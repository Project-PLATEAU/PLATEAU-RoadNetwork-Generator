#include "CBridgeDataUtil.h"
#include "CConvertGeometryUtil.h"
#include "CGDALUtil.h"
#include "CLogger.h"

// 橋梁品質情報のlodType(LOD2の場合の詳細度)を取得する
bool CBridgeDataUtil::GetBridDataQualityAttributeLodType(
    const citygml::CityObject *pBridge, double &dLodType)
{
    bool bRet = false;

    if (pBridge != nullptr)
    {
        auto it = pBridge->getAttributes().find(KEY_BRID_DATA_QUALITY_ATTR);
        if (it != pBridge->getAttributes().end())
        {
            for (auto val : it->second.asAttributeSet())
            {
                if (val.first == KEY_BRID_DATA_QUALITY_ATTR_LOD_TYPE)
                {
                    bRet = true;
                    std::string strVal = val.second.asString();
                    if (strVal.compare(KEY_BRID_DATA_QUALITY_ATTR_LOD_TYPE_20) == 0)
                    {
                        dLodType = 2.0;
                    }
                    else if (strVal.compare(KEY_BRID_DATA_QUALITY_ATTR_LOD_TYPE_21) == 0)
                    {
                        dLodType = 2.1;
                    }
                    break;
                }
            }
        }
    }

    return bRet;
}

// CityObjectからbrid:functionを取得
bool CBridgeDataUtil::GetBridFunction(
    const citygml::CityObject *pBridge,
    CBridgeData &bridgeData)
{
    bool bRet = false;
    bridgeData.m_functionType = BRIDGE_FUNCTION_TYPE::UNKNOWN;

    if (pBridge != nullptr)
    {
        auto it = pBridge->getAttributes().find(KEY_BRID_FUNCTION);
        if (it != pBridge->getAttributes().end())
        {
            std::string strFunction = it->second.asString();
            int nFunction = atoi(strFunction.c_str());

            switch (nFunction)
            {
            case static_cast<std::underlying_type<BRIDGE_FUNCTION_TYPE>::type>(BRIDGE_FUNCTION_TYPE::ROAD_BRIDGE):
                bridgeData.m_functionType = BRIDGE_FUNCTION_TYPE::ROAD_BRIDGE;
                break;
            case static_cast<std::underlying_type<BRIDGE_FUNCTION_TYPE>::type>(BRIDGE_FUNCTION_TYPE::RAILROAD_BRIDGE):
                bridgeData.m_functionType = BRIDGE_FUNCTION_TYPE::RAILROAD_BRIDGE;
                break;
            case static_cast<std::underlying_type<BRIDGE_FUNCTION_TYPE>::type>(BRIDGE_FUNCTION_TYPE::AQUADUCT_BRIDGE):
                bridgeData.m_functionType = BRIDGE_FUNCTION_TYPE::AQUADUCT_BRIDGE;
                break;
            case static_cast<std::underlying_type<BRIDGE_FUNCTION_TYPE>::type>(BRIDGE_FUNCTION_TYPE::CABLE_BRIDGE):
                bridgeData.m_functionType = BRIDGE_FUNCTION_TYPE::CABLE_BRIDGE;
                break;
            case static_cast<std::underlying_type<BRIDGE_FUNCTION_TYPE>::type>(BRIDGE_FUNCTION_TYPE::BRIDGESIDE_PEDESTRIAN_BRIDGE):
                bridgeData.m_functionType = BRIDGE_FUNCTION_TYPE::BRIDGESIDE_PEDESTRIAN_BRIDGE;
                break;
            case static_cast<std::underlying_type<BRIDGE_FUNCTION_TYPE>::type>(BRIDGE_FUNCTION_TYPE::CANAL_BRIDGE):
                bridgeData.m_functionType = BRIDGE_FUNCTION_TYPE::CANAL_BRIDGE;
                break;
            case static_cast<std::underlying_type<BRIDGE_FUNCTION_TYPE>::type>(BRIDGE_FUNCTION_TYPE::PEDESTRIAN_CROSSING_BRIDGE):
                bridgeData.m_functionType = BRIDGE_FUNCTION_TYPE::PEDESTRIAN_CROSSING_BRIDGE;
                break;
            case static_cast<std::underlying_type<BRIDGE_FUNCTION_TYPE>::type>(BRIDGE_FUNCTION_TYPE::PEDESTRIAN_DECK):
                bridgeData.m_functionType = BRIDGE_FUNCTION_TYPE::PEDESTRIAN_DECK;
            break;
                default:
                break;
            }

            bRet = true;
        }
    }
    return bRet;
}

// 歩道橋の場合CityObjectから橋梁データを取得する
bool CBridgeDataUtil::GetPedestrianCrossingBridge(
    const citygml::CityObject *pBridge,
    CBridgeData &bridgeData,
    const int nJPZone)
{
    bool bRet = false;

    if (pBridge != nullptr)
    {
        // 橋梁の種別を取得
        if (GetBridFunction(pBridge, bridgeData))
        {
            if (bridgeData.m_functionType == BRIDGE_FUNCTION_TYPE::PEDESTRIAN_CROSSING_BRIDGE)
            {
                // 横断歩道橋の場合
                bridgeData.m_strId = pBridge->getId();
                bool bLOD1 = getBridgeDataLOD1(pBridge, bridgeData, nJPZone);
                bool bLOD2 = getBridgeDataLOD2(pBridge, bridgeData, nJPZone);
                bRet = (bLOD1 || bLOD2);
            }
        }
    }

    return bRet;
}

// LOD1橋梁データの取得
bool CBridgeDataUtil::getBridgeDataLOD1(
    const citygml::CityObject *pBridge,
    CBridgeData &bridgeData,
    const int nJPZone)
{
    if (pBridge != nullptr)
    {
        for (uint32_t geoIdx = 0; geoIdx < pBridge->getGeometriesCount(); geoIdx++)
        {
            auto geo = pBridge->getGeometry(geoIdx);

            if (geo.getLOD() == static_cast<uint32_t>(LOD_TYPE::LOD1)
                && geo.getGeometriesCount() > 0)
            {
                for (uint32_t lod1GeoIdx = 0; lod1GeoIdx < geo.getGeometriesCount(); lod1GeoIdx++)
                {
                    // 箱モデルを全て取得(上面 or 底面だけで良いかも)
                    auto subGeo = geo.getGeometry(lod1GeoIdx);
                    for (uint32_t polyIdx = 0; polyIdx < subGeo.getPolygonsCount(); polyIdx++)
                    {
                        std::shared_ptr<const citygml::Polygon> pPoly = subGeo.getPolygon(polyIdx);
                        auto polygon = CConvertGeometryUtil::ConvBoostPolygon(pPoly, nJPZone);

                        if (!bg::is_empty(polygon))
                        {
                            CBridgeDataLod1 lod1Data;
                            lod1Data.m_boostGeometry = polygon;

                            bridgeData.m_lod1List.emplace_back(lod1Data);
                        }
                    }
                }
            }
        }
    }

    return (bridgeData.m_lod1List.size() > 0);
}

 // LOD2橋梁データの取得
bool CBridgeDataUtil::getBridgeDataLOD2(
    const citygml::CityObject *pBridge,
    CBridgeData &bridgeData,
    const int nJPZone)
{
    if (pBridge != nullptr)
    {
        // LOD2詳細度取得(LOD2以上は必須)
        double lodType = 0.0;
        CBridgeDataUtil::GetBridDataQualityAttributeLodType(pBridge, lodType);
        bridgeData.m_dLod2Type = lodType;

        // 橋梁の外形形状を取得したいため、LOD2ポリゴンを融合する
        // 標高値を取得するため底面に相当する三角メッシュポリゴンは別途保持する
        Boost3DHashMultiPolygon tmpPolygons, dissolvePolygons;
        for (uint32_t childIdx = 0; childIdx < pBridge->getChildCityObjectsCount(); childIdx++)
        {
            auto &childObj = pBridge->getChildCityObject(childIdx);
            bool bSaveMesh = false;
            if (childObj.getTypeAsString().compare(KEY_BRID_PART_TYPE_OUTER_FLOOR_SURFACE) == 0
                || childObj.getTypeAsString().compare(KEY_BRID_PART_TYPE_OUTER_CEILING_SURFACE) == 0)
            {
                bSaveMesh = true;
            }
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
                            tmpPolygons.push_back(polygon); // 外形形状を取得したいため融解用データに追加

                            if (bSaveMesh)
                            {
                                // 標高値取得用の底面三角メッシュポリゴン
                                CBridgeDataLod2 mesh;
                                mesh.m_boostGeometry = polygon;
                                bridgeData.m_lod2TriangularMeshList.emplace_back(mesh);
                            }
                        }
                    }
                }
            }
        }
        if (tmpPolygons.size() > 1)
        {
            dissolvePolygons = CGDALUtil::GetInstance()->Dissolve(
                tmpPolygons, true, true, ncl_common_def::POINT_SIGNIFICANT_DIGITS_FOR_DISSOLVE);
        }
        else
        {
            dissolvePolygons = tmpPolygons;
        }
        if (!bg::is_empty(dissolvePolygons))
        {
            for (const auto &polygon : dissolvePolygons)
            {
                CBridgeDataLod2 childData;
                childData.m_boostGeometry = polygon;
                bridgeData.m_lod2List.emplace_back(childData);
            }
        }
    }
    return (bridgeData.m_lod2List.size() > 0);
}
