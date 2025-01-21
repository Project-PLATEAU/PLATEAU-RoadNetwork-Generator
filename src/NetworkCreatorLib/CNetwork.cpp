#include "CNetwork.h"
#include "boost/foreach.hpp"
#include "boost/graph/dijkstra_shortest_paths.hpp"
#include "boost/format.hpp"
#include "CFileUtil.h"
#include "CGeoUtil.h"
#include "CStatusManager.h"
#include "CTime.h"
#include "SettingData.h"
#include "CBoostGeoUtil.h"
#include "COpenCVUtil.h"
#include "CTranRoadDataUtil.h"
#include <thread>
#include "CDebugUtil.h"
#include "CLogger.h"
#include "CErrLogger.h"

// コンストラクタ
CNetwork::CNetwork(const NETWORK_DATA_TYPE type, const double dLod3Detail)
    : m_dataType(type),
      m_nNodeIdDigit(30),
      m_nLinkIdDigit(10),    // TODO : 仮
      m_nLanLotDigit(19),
      m_nLanLotDecimal(15),
      m_dLod3Detail(dLod3Detail)
{

}

// ネットワーク追加(中心線1本分)
void CNetwork::add(
    const std::shared_ptr<CCenterLineData> &centerLinePtr,
    const std::shared_ptr<CTranRoadData> &roadPtr,
    const std::shared_ptr<CFurnitureData> &frnPtr,
    const std::shared_ptr<CBridgeData> &bridgePtr)
{
    bool bRegistration = false;

    if (centerLinePtr != nullptr
        && (roadPtr != nullptr
        || (frnPtr != nullptr && frnPtr->m_pedestrianCrossingData.m_bUse)
        || (bridgePtr != nullptr && bridgePtr->m_pedestrianBridgeData.m_bUse)))
        bRegistration = true;   // 中心線が存在、かつ、道路または有効な都市設備/橋梁の場合は登録する

    if (bRegistration)
    {
        BoostVertexDesc prevDesc = BoostUndirectedGraph::null_vertex();
        for (Boost3DHashPolyline::const_iterator it = centerLinePtr->centerLine.cbegin();
            it != centerLinePtr->centerLine.cend(); it++)
        {
            if (prevDesc != BoostUndirectedGraph::null_vertex()
                && CEpsUtil::Zero(m_graph[prevDesc].pt.RoundDistance(*it)))
            {
                // 同一頂点の場合はスキップ
                continue;
            }

            // 既存点の確認
            double dDist;
            BoostVertexDesc targetDesc = NNSearch(*it, dDist);  // 最近傍点探索
            if (targetDesc == BoostUndirectedGraph::null_vertex()
                ||(targetDesc != BoostUndirectedGraph::null_vertex() && CEpsUtil::Greater(dDist, it->dEpsilon)))
            {
                // 未登録の場合は頂点の追加
                BoostVertexProperty v(*it, roadPtr, frnPtr, bridgePtr, centerLinePtr);
                BoostVertexDesc desc = boost::add_vertex(v, m_graph);
                m_graph[desc].desc = desc;
                targetDesc = desc;
                // 近傍探索用RTree
                m_vertexRTree.insert(VertexRTreeValue(*it, desc));
            }
            else
            {
                // 既存の場合は情報更新
                m_graph[targetDesc].AddCenterLinePtr(centerLinePtr);
                m_graph[targetDesc].AddRoadPtr(roadPtr);
                m_graph[targetDesc].AddFurniturePtr(frnPtr);
                m_graph[targetDesc].AddBridgePtr(bridgePtr);
            }

            if (prevDesc != BoostUndirectedGraph::null_vertex())
            {
                // エッジの追加
                auto edge = boost::add_edge(prevDesc, targetDesc, m_graph);
                m_graph[edge.first].vertexDesc1 = prevDesc;
                m_graph[edge.first].vertexDesc2 = targetDesc;
                m_graph[edge.first].dLength = m_graph[prevDesc].pt.RoundDistance(*it);
                m_graph[edge.first].srcRoadPtr = roadPtr;
                m_graph[edge.first].srcFrnPtr = frnPtr;
                m_graph[edge.first].srcBridgePtr = bridgePtr;
                m_graph[edge.first].srcCenterLinePtr = centerLinePtr;
            }
            prevDesc = targetDesc;
        }
        // 道路情報一括管理マップの更新
        updateRoadHashMap(roadPtr, centerLinePtr);
        // 都市設備情報一括管理マップの更新
        updateFurnitureHashMap(frnPtr, centerLinePtr);
        // 橋梁情報一括管理マップの更新
        updateBridgeHashMap(bridgePtr, centerLinePtr);
    }
}

// ネットワーク追加(道路中心線複数本分)
void CNetwork::add(
    const std::vector<std::shared_ptr<CCenterLineData>> &vecCenterLine,
    const std::shared_ptr<CTranRoadData> &roadPtr)
{
    for (const auto &centerLine : vecCenterLine)
        add(centerLine, roadPtr, nullptr, nullptr);
}

// ネットワーク追加(複数道路一括)
void CNetwork::Add(const std::vector<std::shared_ptr<CTranRoadData>> &vecRoad)
{
    for (const auto &roadPtr : vecRoad)
    {
        if (m_dataType == NETWORK_DATA_TYPE::ROADWAY)
        {
            if (roadPtr->m_bIsCenterLineOnNeighborRoad)
                continue;   // 自身が交差点かつ近傍道路も交差点の場合で、近傍交差点側に中心線が設定されている場合は無視

            if (roadPtr->roadCenterLineList.size() > 0)
            {
                add(roadPtr->roadCenterLineList, roadPtr);
            }
            else
            {
                // 中心線が存在しない
                if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD1
                    && !bg::is_empty(roadPtr->m_lod1.m_boostGeometry))
                {
                    CErrLogger::GetInstance()->WriteRoadwayLog(
                        RoadwayErrType::GET_LINK_FAILED,
                        CErrLogger::GetInstance()->GetRefPt(roadPtr->m_lod1.m_boostGeometry));
                }
                else if ((CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD2))
                {
                    for (const auto &poly : roadPtr->m_lod2List)
                    {
                        if (poly.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY)
                            || poly.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION))
                        {
                            CErrLogger::GetInstance()->WriteRoadwayLog(
                                RoadwayErrType::GET_LINK_FAILED,
                                CErrLogger::GetInstance()->GetRefPt(poly.m_boostGeometry));
                            break;
                        }
                    }
                }
                else if ((CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3))
                {
                    for (const auto &poly : roadPtr->m_lod3List)
                    {
                        if (poly.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY)
                            || poly.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::LANE)
                            || poly.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION))
                        {
                            CErrLogger::GetInstance()->WriteRoadwayLog(
                                RoadwayErrType::GET_LINK_FAILED,
                                CErrLogger::GetInstance()->GetRefPt(poly.m_boostGeometry));
                            break;
                        }
                    }
                }
            }
        }
        else if (m_dataType == NETWORK_DATA_TYPE::FOOTPATH)
        {
            if (roadPtr->m_footpath.centerLineList.size() > 0)
            {
                add(roadPtr->m_footpath.centerLineList, roadPtr);
            }
            else
            {
                // 中心線が存在しない
                if ((CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD2))
                {
                    for (const auto &poly : roadPtr->m_lod2List)
                    {
                        if (poly.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH))
                        {
                            if (roadPtr->m_nInOut >= 3 && CEpsUtil::LessEqual(CBoostGeoUtil::Area(poly.m_boostGeometry), 1.0))
                                continue;   // 交差点内の小面積歩道は無視

                            CErrLogger::GetInstance()->WriteFootpathLog(
                                FootpathErrType::GET_LINK_FAILED,
                                CErrLogger::GetInstance()->GetRefPt(poly.m_boostGeometry));
                            break;
                        }
                    }
                }
                else if ((CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3))
                {
                    for (const auto &poly : roadPtr->m_lod3List)
                    {
                        if (poly.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH))
                        {
                            if (roadPtr->m_nInOut >= 3 && CEpsUtil::LessEqual(CBoostGeoUtil::Area(poly.m_boostGeometry), 1.0))
                                continue;   // 交差点内の小面積歩道は無視

                            CErrLogger::GetInstance()->WriteFootpathLog(
                                FootpathErrType::GET_LINK_FAILED,
                                CErrLogger::GetInstance()->GetRefPt(poly.m_boostGeometry));
                            break;
                        }
                    }
                }
            }
        }
    }

    if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
    {
        // 勾配計測用の三角メッシュ登録
        m_sor.SetData(vecRoad, CInputSettingData::GetInstance()->lodType);
    }
}

// ネットワーク追加(複数都市設備一括)
void CNetwork::Add(
    const std::vector<std::shared_ptr<CFurnitureData>> &vecFurniture)
{
    for (const auto &frnPtr : vecFurniture)
    {
        if (frnPtr->m_pedestrianCrossingData.m_bUse)
        {
            add(frnPtr->m_pedestrianCrossingData.GetCenterLine(), nullptr, frnPtr, nullptr);
        }
    }
}

// ネットワーク追加(複数橋梁一括)
void CNetwork::Add(const std::vector<std::shared_ptr<CBridgeData>> &vecBridge)
{
    for (const auto &bridgePtr : vecBridge)
    {
        if (bridgePtr->m_pedestrianBridgeData.m_bUse)
        {
            for (const auto &centerLine : bridgePtr->m_pedestrianBridgeData.m_centerLines)
            {
                std::shared_ptr<CCenterLineData> centerLinePtr = std::make_shared<CCenterLineData>(centerLine.m_centerLine);
                add(centerLinePtr, nullptr, nullptr, bridgePtr);
            }
        }
    }

    if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
    {
        // 勾配計測用の三角メッシュ登録
        m_sob.SetData(vecBridge);
    }
}

// 点字ブロック情報の設定
void CNetwork::SetBrailleTile(
    const std::vector<std::shared_ptr<CFurnitureData>> &vecFurniture,
    const double dInterval,
    const double dLengthTh)
{
    m_brailleTileRTree.clear();

    // 最近傍点字ブロック探索準備
    for (const auto &frnPtr : vecFurniture)
    {
        if (frnPtr->m_functionType == FURNITURE_FUNCTION_TYPE::BRAILLE_BLOCKS)
        {

            // 使用するポリゴンの選択(LOD3優先)
            std::vector<std::shared_ptr<Boost3DHashPolygon>> polygons;
            if (frnPtr->m_lod3List.size() > 0)
            {
                for (const auto &lod3 : frnPtr->m_lod3List)
                    polygons.emplace_back(std::make_shared<Boost3DHashPolygon>(lod3.m_boostGeometry));
            }
            else if (frnPtr->m_lod2List.size() > 0)
            {
                for (const auto &lod2 : frnPtr->m_lod2List)
                    polygons.emplace_back(std::make_shared<Boost3DHashPolygon>(lod2.m_boostGeometry));
            }

            for (const auto &polygon : polygons)
            {
                BoostPolygon tmpPoly;
                bg::simplify(CBoostGeoUtil::Conv(*polygon), tmpPoly, 0.1);
                if (bg::is_empty(tmpPoly))
                    continue;

                Boost3DHashPolygon tmpPoly2;
                for (const auto &pt : tmpPoly.outer())
                    tmpPoly2.outer().push_back(Boost3DPointHash(pt.x(), pt.y(), 0));
                std::vector<std::tuple<Boost3DHashPolyline, bool, bool>> lines = COpenCVUtil::Thinning(tmpPoly2, 0.1, 0.3, 0.5);
                for (const auto &val : lines)
                {
                    auto [line, bFront, bBack] = val;
                    if (CEpsUtil::GreaterEqual(bg::length(line), dLengthTh))
                    {
                        Boost3DHashPolyline samplingLine = CBoostGeoUtil::Sampling(line, dInterval);
                        for (const auto &pt : samplingLine)
                            m_brailleTileRTree.insert(BrailleTileTuple(pt, frnPtr, polygon));
                    }
                }
            }
        }
    }
}

// グラフのクリア
void CNetwork::Clear()
{
    m_roadHashMap.clear();
    m_frnHashMap.clear();
    m_bridgeHashMap.clear();
    m_graph.clear();
    m_vertexRTree.clear();
    m_brailleTileRTree.clear();
    m_sor.Clear();
    m_sob.Clear();
}

// 最近傍頂点の探索
BoostVertexDesc CNetwork::NNSearch(const Boost3DPointHash &pt, double &dDist)
{
    dDist = 0;
    BoostVertexDesc desc = BoostUndirectedGraph::null_vertex();
    std::vector<VertexRTreeValue> vec;
    m_vertexRTree.query(bg::index::nearest(pt, 1), std::back_inserter(vec));
    if (vec.size() > 0)
    {
        desc = vec[0].second;
        dDist = pt.RoundDistance(vec[0].first);
    }
    return desc;
}

// ネットワークデータ出力
void CNetwork::OutputNetworkData(
    const std::string &strShpOutputFolder,
    const std::string &strGeoJsonOutputFolder,
    const int nJPZone,
    const bool isUseZ,
    const OUTPUT_FILE_TYPE fileType,
    const std::string strEncoding)
{
    // EPSGコード
    int nEpsg = isUseZ ? CEpsgUtil::AsInt(CEpsgUtil::EPSGCode::EPSG_JGD2011_VERTICAL_HEIGHT) : CEpsgUtil::AsInt(CEpsgUtil::EPSGCode::EPSG_JGD2011);

    // 出力ファイルパス
    std::string strShpNodeFilePath, strShpLinkFilePath;
    std::string strGeoJsonNodeFilePath, strGeoJsonLinkFilePath;
    if (fileType == CNetwork::OUTPUT_FILE_TYPE::GEOJSON || fileType == CNetwork::OUTPUT_FILE_TYPE::BOTH)
    {
        strGeoJsonNodeFilePath = CFileUtil::GetInstance()->Combine(strGeoJsonOutputFolder, "node.geojson");
        strGeoJsonLinkFilePath = CFileUtil::GetInstance()->Combine(strGeoJsonOutputFolder, "link.geojson");
    }

    if (fileType == CNetwork::OUTPUT_FILE_TYPE::SHP || fileType == CNetwork::OUTPUT_FILE_TYPE::BOTH)
    {
        strShpNodeFilePath = CFileUtil::GetInstance()->Combine(strShpOutputFolder, "node.shp");
        strShpLinkFilePath = CFileUtil::GetInstance()->Combine(strShpOutputFolder, "link.shp");
    }

    // 出力データ作成
    std::vector<Node> nodes;
    std::vector<Link> links;
    int nMaxLinkNum;
    getNetworkData(nJPZone, nodes, links, nMaxLinkNum);

    // リンクデータ
    Boost3DHashMultiLines polylines;    // 幾何情報
    std::vector<CGISFileAttribute::AttributeDataRecord> linkAttrRecords;    // 属性データ
    createLinkData(links, m_dataType, nJPZone, polylines, linkAttrRecords);
    std::vector<CGISFileAttribute::AttributeFieldData> linkFields = createLinkFields(m_dataType); // 属性フィールド情報

    // ノードデータ
    Boost3DMultiPointHashs pts; // 幾何情報
    std::vector<CGISFileAttribute::AttributeDataRecord> nodeAttrRecords;    // 属性データ
    createNodeData(nodes, nMaxLinkNum, m_dataType, nJPZone, pts, nodeAttrRecords);
    std::vector<CGISFileAttribute::AttributeFieldData> nodeFields = createNodeFields(nMaxLinkNum, m_dataType); // 属性フィールド情報

    // ファイル出力
    if (fileType == CNetwork::OUTPUT_FILE_TYPE::GEOJSON || fileType == CNetwork::OUTPUT_FILE_TYPE::BOTH)
    {
        CGISFileExporter exporter(CGISFileExporter::GIS_FILE_TYPE::GEOJSON);
        exporter.OutputPolylines(
            polylines, strGeoJsonLinkFilePath, linkFields, linkAttrRecords,
            isUseZ, nEpsg, strEncoding);
        exporter.OutputMultiPoints(
            pts, strGeoJsonNodeFilePath, nodeFields, nodeAttrRecords,
            isUseZ, nEpsg, strEncoding);
    }
    if (fileType == CNetwork::OUTPUT_FILE_TYPE::SHP || fileType == CNetwork::OUTPUT_FILE_TYPE::BOTH)
    {
        CGISFileExporter exporter(CGISFileExporter::GIS_FILE_TYPE::SHP);
        exporter.OutputPolylines(
            polylines, strShpLinkFilePath, linkFields, linkAttrRecords,
            isUseZ, nEpsg, strEncoding);
        exporter.OutputMultiPoints(
            pts, strShpNodeFilePath, nodeFields, nodeAttrRecords,
            isUseZ, nEpsg, strEncoding);
    }
}

// 関連道路情報管理用ハッシュマップの更新
void CNetwork::updateRoadHashMap(
    const std::shared_ptr<CTranRoadData> &roadPtr,
    const std::shared_ptr<CCenterLineData> &centerLinePtr)
{
    if (roadPtr == nullptr)
        return;

    auto itRoad = m_roadHashMap.find(roadPtr);
    if (itRoad == m_roadHashMap.end())
    {
        // 新規作成
        std::set<std::shared_ptr<CCenterLineData>> val;
        val.insert(centerLinePtr);
        m_roadHashMap.insert(RoadHashMap::value_type(roadPtr, val));
    }
    else
    {
        // 追加
        itRoad->second.insert(centerLinePtr);
    }
}

// 関連都市設備情報管理用ハッシュマップの更新
void CNetwork::updateFurnitureHashMap(
    const std::shared_ptr<CFurnitureData> &frnPtr,
    const std::shared_ptr<CCenterLineData> &centerLinePtr)
{
    if (frnPtr == nullptr)
        return;

    auto itFurniture = m_frnHashMap.find(frnPtr);
    if (itFurniture == m_frnHashMap.end())
    {
        // 新規作成
        std::set<std::shared_ptr<CCenterLineData>> val;
        val.insert(centerLinePtr);
        m_frnHashMap.insert(FurnitureHashMap::value_type(frnPtr, val));
    }
    else
    {
        // 追加
        itFurniture->second.insert(centerLinePtr);
    }
}

// 関連橋梁情報管理用ハッシュマップの更新
void CNetwork::updateBridgeHashMap(
    const std::shared_ptr<CBridgeData> &bridgePtr,
    const std::shared_ptr<CCenterLineData> &centerLinePtr)
{
    if (bridgePtr == nullptr)
        return;

    auto itBridge = m_bridgeHashMap.find(bridgePtr);
    if (itBridge == m_bridgeHashMap.end())
    {
        // 新規作成
        std::set<std::shared_ptr<CCenterLineData>> val;
        val.insert(centerLinePtr);
        m_bridgeHashMap.insert(BridgeHashMap::value_type(bridgePtr, val));
    }
    else
    {
        // 追加
        itBridge->second.insert(centerLinePtr);
    }
}

// ネットワークデータ状のノード点確認
bool CNetwork::isNode(const BoostVertexDesc &v)
{
    // ノード点の条件
    // 隣接道路,横断歩道,横断歩道橋の境界点 or 終端道路の端点 or エッジの分岐点

    // 端点確認
    int nEdgeCnt = 0;
    BOOST_FOREACH(const auto &desc, boost::out_edges(v, m_graph))
    {
        nEdgeCnt++;
    }

    // 境界点確認
    size_t ptrCnt = m_graph[v].srcRoads.size() + m_graph[v].srcFurnitures.size() + m_graph[v].srcBridges.size();

    return (ptrCnt > 1 || nEdgeCnt == 1 || nEdgeCnt > 2);
}

// ID生成
std::string CNetwork::getId(
    const int nJPZone,
    const Boost3DPointHash &pt,
    std::map<std::string, std::map<Boost3DPointHash, int>> &map)
{
    // 平面直角座標系の系番号2桁
    // 符号1桁(正:0, 負:1) + 座標の整数部6桁 (xyz分)
    // 小数部は6桁(連番用)
    // 小数点含め, 計30文字
    std::string strId = (boost::format("%02d") % abs(nJPZone)).str();
    int nXSign = CEpsUtil::Less(pt.x(), 0) ? 1 : 0;
    int nYSign = CEpsUtil::Less(pt.y(), 0) ? 1 : 0;
    int nZSign = CEpsUtil::Less(pt.z(), 0) ? 1 : 0;
    strId += (boost::format("%1d%06d") % nXSign % static_cast<int>(floor(abs(pt.x())))).str();
    strId += (boost::format("%1d%06d") % nYSign % static_cast<int>(floor(abs(pt.y())))).str();
    strId += (boost::format("%1d%06d") % nZSign % static_cast<int>(floor(abs(pt.z())))).str();

    // 連番付与
    int nNum = 0;
    auto itTargetId = map.find(strId);
    if (itTargetId == map.end())
    {
        // IDが未登録の場合
        std::map<Boost3DPointHash, int> val;
        val.insert(std::map<Boost3DPointHash, int>::value_type(pt, nNum));
        map.insert(std::map<std::string, std::map<Boost3DPointHash, int>>::value_type(strId, val));
    }
    else
    {
        auto itTargetPt = itTargetId->second.find(pt);
        if (itTargetPt != itTargetId->second.end())
        {
            // 既存点の場合
            nNum = itTargetPt->second;
        }
        else
        {
            // 連番追加の場合
            nNum = static_cast<int>(itTargetId->second.size());
            itTargetId->second.insert(std::pair<Boost3DPointHash, int>(pt, nNum));
        }
    }
    strId += (boost::format(".%06d") % nNum).str();    // 左詰め0埋め

    return strId;

}

// ネットワークデータの取得
void CNetwork::getNetworkData(
    const int nJPZone,
    std::vector<Node> &nodes,
    std::vector<Link> &links,
    int &nMaxLinkNum)
{
    nodes.clear();
    links.clear();
    nMaxLinkNum = 0;


    std::map<std::string, std::map<Boost3DPointHash, int>> linkIdMap;   // リンクIDの連番管理用
    std::map<std::string, std::map<Boost3DPointHash, int>> nodeIdMap;   // ノードIDの連番管理用
    std::unordered_map<BoostVertexDesc, Node> nodeDataMap;
    std::vector<std::thread> threads;

    // 道路ごとにリンクデータを作成する
    for (const auto &targetRoad : m_roadHashMap)
    {
        threads.emplace_back([targetRoad, &linkIdMap, &nodeIdMap, &nodeDataMap, nJPZone, &links, this]()
        {
            Boost3DHashMultiLines geoms;
            for (const auto &centerLine : targetRoad.second)
            {
                getLink(centerLine, targetRoad.first, nullptr, nullptr, nJPZone,
                    links, linkIdMap, nodeIdMap, nodeDataMap, geoms);
            }

            // リンク作成結果の確認
            Boost3DMultiPointHashs pts = checkLink(targetRoad.first, geoms);
            if (pts.size() > 0)
            {
                std::lock_guard<std::mutex> lock(m_errLogMutex);
                for (const auto &pt : pts)
                {
                    if (m_dataType == NETWORK_DATA_TYPE::ROADWAY)
                        CErrLogger::GetInstance()->WriteRoadwayLog(RoadwayErrType::GET_LINK_FAILED, pt);
                    else
                        CErrLogger::GetInstance()->WriteFootpathLog(FootpathErrType::GET_LINK_FAILED, pt);
                }
            }

        });
    }

    // 都市設備(横断歩道)
    for (const auto &targetFrn : m_frnHashMap)
    {
        threads.emplace_back([targetFrn, &linkIdMap, &nodeIdMap, &nodeDataMap, nJPZone, &links, this]()
        {
            for (const auto &centerLine : targetFrn.second)
            {
                Boost3DHashMultiLines geoms;
                getLink(centerLine, nullptr, targetFrn.first, nullptr,
                    nJPZone, links, linkIdMap, nodeIdMap, nodeDataMap, geoms);
            }
        });
    }

    // 橋梁(横断歩道橋)
    for (const auto &targetBridge : m_bridgeHashMap)
    {
        threads.emplace_back([targetBridge, &linkIdMap, &nodeIdMap, &nodeDataMap, nJPZone, &links, this]()
        {
            for (const auto &centerLine : targetBridge.second)
            {
                Boost3DHashMultiLines geoms;
                getLink(centerLine, nullptr, nullptr, targetBridge.first, nJPZone,
                    links, linkIdMap, nodeIdMap, nodeDataMap, geoms);
            }
        });
    }

    for (auto &th : threads)
        th.join();

    // ノード情報の作成
    for (const auto &v : nodeDataMap)
    {
        if (v.second.edges.size() > nMaxLinkNum)
            nMaxLinkNum = static_cast<int>(v.second.edges.size());

        nodes.push_back(v.second);
    }
}

// ネットワークデータのリンク取得(マルチスレッド対応入り)
bool CNetwork::getLink(
    const std::shared_ptr<CCenterLineData> &centerLinePtr,
    const std::shared_ptr<CTranRoadData> &roadPtr,
    const std::shared_ptr<CFurnitureData> &frnPtr,
    const std::shared_ptr<CBridgeData> &bridgePtr,
    const int nJPZone,
    std::vector<Link> &links,
    std::map<std::string, std::map<Boost3DPointHash, int>> &linkIdMap,
    std::map<std::string, std::map<Boost3DPointHash, int>> &nodeIdMap,
    std::unordered_map<BoostVertexDesc, Node> &nodeDataMap,
    Boost3DHashMultiLines &geoms)
{
    const double dRoadwayVSlopeInterval = 10.0;         // 車道用縦断勾配のサンプリング間隔
    const double dFootpathVSlopeInterval = 10.0;        // 歩道用縦断勾配のサンプリング間隔
    const double dMinWidthThForVSlope = 1.0;            // 縦断勾配計算対象となる道路の最小幅員しきい値
    const double dFootpathHSlopeInterval = 1.0;         // 横断勾配のサンプリング間隔
    const double dMinWidthThForHSlope = 1.0;            // 横断勾配計算対象となる道路の最小幅員しきい値

    bool isAdd = false;

    // 作成日
    std::string strCreateDate = CTime::GetCurrentTime().Format("%Y-%m-%d");

    // 注目道路or都市設備(横断歩道)or橋梁(横断歩道橋)に関連する頂点を収集
    std::set<VertexRTreeValue> targetVertices;  // 注目道路の全頂点
    std::set<VertexRTreeValue> targetNodes;     // 注目道路の頂点の内、隣接道路の境界点 or 終端道路の端点
    std::set<BoostEdgeDesc> targetEdges;        // 注目道路のエッジ

    if (roadPtr != nullptr)
    {
        // 道路の場合
        m_vertexRTree.query(
            bg::index::satisfies([roadPtr, centerLinePtr, this](const VertexRTreeValue &v)
                { return this->CheckVertex(v, roadPtr, centerLinePtr); }),
            std::inserter(targetVertices, targetVertices.begin()));
    }
    else if (frnPtr != nullptr)
    {
        // 都市設備の場合
        m_vertexRTree.query(
            bg::index::satisfies([frnPtr, centerLinePtr, this](const VertexRTreeValue &v)
                { return this->CheckVertex(v, frnPtr, centerLinePtr); }),
            std::inserter(targetVertices, targetVertices.begin()));
    }
    else if (bridgePtr != nullptr)
    {
        // 橋梁の場合
        m_vertexRTree.query(
            bg::index::satisfies([bridgePtr, centerLinePtr, this](const VertexRTreeValue &v)
                { return this->CheckVertex(v, bridgePtr, centerLinePtr); }),
            std::inserter(targetVertices, targetVertices.begin()));
    }

    for (const auto &v : targetVertices)
    {
        // ノード点の判定
        if (isNode(v.second))
            targetNodes.insert(v);

        // 注目道路のエッジ収集
        BOOST_FOREACH(BoostEdgeDesc edgeDesc, boost::out_edges(v.second, m_graph))
        {
            if (roadPtr != nullptr)
            {
                // 道路の場合
                if (m_graph[edgeDesc].srcRoadPtr == roadPtr)
                    targetEdges.insert(edgeDesc);
            }
            else if (frnPtr != nullptr)
            {
                // 都市設備の場合
                if (m_graph[edgeDesc].srcFrnPtr == frnPtr)
                    targetEdges.insert(edgeDesc);
            }
            else if (bridgePtr != nullptr)
            {
                // 橋梁の場合
                if (m_graph[edgeDesc].srcBridgePtr == bridgePtr)
                    targetEdges.insert(edgeDesc);
            }
        }
    }

    //データ構造上、道路に対してノード点は2点以上
    if (targetNodes.size() < 2)
    {
        // リンク線の作成失敗
        return isAdd;
    }

    // 注目道路のエッジのみでグラフを作成
    BoostUndirectedGraph subGraph;
    // サブグラフの頂点と全体グラフの頂点の対照マップ
    std::unordered_map<BoostVertexDesc, BoostVertexDesc> globalToLocalVetexMap;

    for (const auto &e : targetEdges)
    {
        // 既存点の確認
        std::vector<BoostVertexDesc> descs, localDescs;
        descs.push_back(m_graph[e].vertexDesc1);
        descs.push_back(m_graph[e].vertexDesc2);
        for (const BoostVertexDesc &desc : descs)
        {
            auto itTarget = globalToLocalVetexMap.find(desc);
            if (itTarget == globalToLocalVetexMap.end())
            {
                // 未登録の場合は頂点の追加
                BoostVertexProperty v(m_graph[desc].pt, roadPtr, frnPtr, bridgePtr, centerLinePtr);
                BoostVertexDesc localDesc = boost::add_vertex(v, subGraph);
                subGraph[localDesc].desc = desc;   // 全体グラフのディスクリプタ

                // 対照マップ
                globalToLocalVetexMap.insert(
                    std::pair<BoostVertexDesc, BoostVertexDesc>(desc, localDesc));

                // エッジ追加用
                localDescs.push_back(localDesc);
            }
            else
            {
                localDescs.push_back(itTarget->second);
            }
        }

        assert(localDescs.size() == 2);

        // エッジの追加
        auto edge = boost::add_edge(localDescs[0], localDescs[1], subGraph);
        subGraph[edge.first].vertexDesc1 = localDescs[0];
        subGraph[edge.first].vertexDesc2 = localDescs[1];
        subGraph[edge.first].dLength = subGraph[localDescs[0]].pt.RoundDistance(subGraph[localDescs[1]].pt);
    }

    // 注目道路のリンクの端点となるノードを使用して経路探索
    std::set<BoostVertexDesc> targetLocalDesc;
    for (const auto &v : targetNodes)
    {
        auto itLocalDesc = globalToLocalVetexMap.find(v.second);
        if (itLocalDesc != globalToLocalVetexMap.end())
        {
            targetLocalDesc.insert(itLocalDesc->second);
        }
    }

    // 路線名(道路のみ)
    std::string strName = (roadPtr != nullptr) ? roadPtr->m_strName : "";

    // 道路区分(車道のみ)
    std::string strFunction = (roadPtr != nullptr && m_dataType == NETWORK_DATA_TYPE::ROADWAY) ? roadPtr->m_strFunction : "";

    // 探索済みデータ
    std::set<std::pair<BoostVertexDesc, BoostVertexDesc>>searchedDesc;
    for (const auto &from : targetLocalDesc)
    {
        // 注目ノード点とその他ノード点との経路探索
        std::vector<BoostVertexDesc> pred(
            boost::num_vertices(subGraph), BoostUndirectedGraph::null_vertex());
        std::vector<double> vecDistance(boost::num_vertices(subGraph));
        boost::dijkstra_shortest_paths(
            subGraph, from,
            boost::predecessor_map(pred.data()).
            distance_map(vecDistance.data()).
            weight_map(boost::get(&BoostEdgeProperty::dLength, subGraph)));

        for (const auto &to : targetLocalDesc)
        {
            if (to == from)
                continue;   // 注目ノード(開始点)はskip

            if (pred[to] == to)
                continue;   // 経路がない場合はskip

            // 最短経路が存在する場合、経路内に他のノード点が存在しないか確認する
            BoostVertexDesc tmpDesc = pred[to];
            for (; tmpDesc != from; tmpDesc = pred[tmpDesc])
            {
                if (targetLocalDesc.find(tmpDesc) != targetLocalDesc.end())
                    break;
            }
            if (tmpDesc != from)
                continue;   // 経路途中にノード点を含む場合はskip

            std::pair<BoostVertexDesc, BoostVertexDesc> r(from, to);
            if (searchedDesc.find(r) != searchedDesc.end())
                continue;   // 取得済み経路のためskip

            // 未取得の経路が存在する場合は登録
            Boost3DHashPolyline route;
            for (BoostVertexDesc tmpDesc = to;
                tmpDesc != from; tmpDesc = pred[tmpDesc])
            {
                route.push_back(subGraph[tmpDesc].pt);
            }
            route.push_back(subGraph[from].pt);
            bg::reverse(route);

            // リンク延長
            double dLength = CUtil::RoundN(vecDistance[to], 1);

            // リンクID
            std::string strLinkId;
            {
                Boost3DPointHash centerPt;
                if (route.size() > 2)
                {
                    centerPt = route[route.size() / 2]; // 3点以上は中央点
                }
                else
                {
                    CVector3D pt1 = CBoostGeoUtil::ToCVector3D(route.front());
                    CVector3D pt2 = CBoostGeoUtil::ToCVector3D(route.back());
                    CVector3D pt = (pt2 - pt1) * 0.5 + pt1;
                    centerPt.x(pt.x);
                    centerPt.y(pt.y);
                    centerPt.z(pt.z);
                }
                std::lock_guard<std::mutex> lock(m_linkIdMutex);
                strLinkId = getId(nJPZone, centerPt, linkIdMap);
            }

            // ノードIDの取得とリンクID設定
            std::vector<std::string> nodeIds;
            std::vector<BoostVertexDesc> localDescs = { from, to };
            for (const auto &desc : localDescs)
            {

                std::string strNodeId;
                {
                    std::lock_guard<std::mutex> lock(m_nodeIdMutex);     // 排他制御
                    strNodeId = getId(nJPZone, subGraph[desc].pt, nodeIdMap);
                }
                nodeIds.push_back(strNodeId);

                {
                    std::lock_guard<std::mutex> lock(m_nodeDataMutex);   // 排他制御
                    auto itTargetNode = nodeDataMap.find(subGraph[desc].desc); // サブグラフの頂点プロパティのdescには全体グラフのディスクリプタを設定中
                    if (itTargetNode == nodeDataMap.end())
                    {
                        // 新規登録
                        std::set<std::string> edges;
                        edges.insert(strLinkId);
                        Node node(strNodeId, subGraph[desc].pt, edges);
                        nodeDataMap.insert(
                            std::pair<BoostVertexDesc, Node>(subGraph[desc].desc, node));
                    }
                    else
                    {
                        // 既存情報の更新
                        itTargetNode->second.edges.insert(strLinkId);
                    }
                }
            }

            // リンクデータの作成
            if (this->m_dataType == CNetwork::NETWORK_DATA_TYPE::ROADWAY)
            {
                bool isValidWidth = centerLinePtr->isIntersection ? false : true;

                double dMinWidth = 0;
                if (isValidWidth)
                {
                    // 最小幅員値(小数点以下1桁で丸める)
                    // 幅員計測自体はリンク線出力よりも前に実施済み
                    dMinWidth = CUtil::RoundN(centerLinePtr->dMinWidth, 1);
                }

                // 縦断勾配計測
                bool isValidVSlope = false;
                VTCL_SLOPE_TYPE vtclSlopeType = VTCL_SLOPE_TYPE::UNKNOWN;
                int nMaxVSlope = 99;
                int nAveVSlope = 99;
                bool isVEndHigher = false;
                Boost3DPointHash maxVSlopePos;
                if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3
                    && !centerLinePtr->isIntersection
                    && isValidWidth && CEpsUtil::Greater(dMinWidth, dMinWidthThForVSlope))
                {
                    // LOD3.0以上かつ交差点のリンクではないかつ最小幅員がしきい値を超える場合
                    isValidVSlope = calcLongitudinalGradient(
                        route, roadPtr, frnPtr, bridgePtr, dRoadwayVSlopeInterval,
                        nMaxVSlope, isVEndHigher, maxVSlopePos, nAveVSlope);

                    // 縦断勾配タイプ
                    if (isValidVSlope)
                    {
                        vtclSlopeType = getVtclSlopeType(nMaxVSlope, isVEndHigher);
                    }
                }

                Link link(strLinkId, route, dLength, isValidWidth, dMinWidth,
                    centerLinePtr->minWidthPos, strCreateDate, nodeIds[0], nodeIds[1],
                    strName, strFunction, isValidVSlope, vtclSlopeType,
                    nMaxVSlope, nAveVSlope);
                {
                    std::lock_guard<std::mutex> lock(m_linkMutex);
                    links.push_back(link);
                    geoms.push_back(link.geom);
                }
                isAdd = true;
            }
            else if (this->m_dataType == CNetwork::NETWORK_DATA_TYPE::FOOTPATH)
            {
                // 最小幅員の計測
                // 横断歩道、横断歩道橋は基になる中心線の幅員を使用
                double dMinWidth;                           // 最小幅員
                Boost3DPointHash minWidthPos;               // 最小幅員地点
                WIDTH_TYPE widthType = WIDTH_TYPE::UNKNOWN; // 幅員コード
                std::string strWidthRank = "X";             // ランク分
                BRAILLE_TILE_TYPE brailleType = BRAILLE_TILE_TYPE::UNKNOWN; // 視覚障害者用の誘導ブロックの有無
                bool isValidWidth = true;
                bool isRefMinWidth = false;
                if (roadPtr != nullptr)
                {
                    // 歩道の場合
                    isValidWidth = measureWidth(
                        route, roadPtr, this->m_dataType, 1.0, dMinWidth, minWidthPos, isRefMinWidth);
                    if (isValidWidth)
                    {
                        dMinWidth = CUtil::RoundN(dMinWidth, 1);    // 最小幅員値(小数点以下1桁で丸める)
                        widthType = getWidthType(dMinWidth);
                        strWidthRank = getWidthRank(dMinWidth);
                        brailleType = checkBrailleTile(route, roadPtr, frnPtr, dMinWidth);
                    }
                }
                else
                {
                    // 横断歩道、横断歩道橋の場合
                    dMinWidth = CUtil::RoundN(centerLinePtr->dMinWidth, 1);
                    minWidthPos = centerLinePtr->minWidthPos;
                    widthType = getWidthType(dMinWidth);
                    strWidthRank = getWidthRank(dMinWidth);
                    brailleType = checkBrailleTile(route, roadPtr, frnPtr, dMinWidth / 2.0);
                }

                // 縦断勾配計測
                bool isValidVSlope = false;
                VTCL_SLOPE_TYPE vtclSlopeType = VTCL_SLOPE_TYPE::UNKNOWN;   // 縦断勾配タイプ
                int nMaxVSlope = 99;
                int nAveVSlope = 99;
                bool isVEndHigher = false;
                Boost3DPointHash maxVSlopePos;
                std::string strVSlopeRank = "X";    // ランク区分
                if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3
                    && isValidWidth && CEpsUtil::Greater(dMinWidth, dMinWidthThForVSlope))
                {
                    // LOD3.0以上かつ交差点のリンクではないかつ最小幅員がしきい値を超える場合)
                    isValidVSlope = calcLongitudinalGradient(
                        route, roadPtr, frnPtr, bridgePtr, dFootpathVSlopeInterval,
                        nMaxVSlope, isVEndHigher, maxVSlopePos, nAveVSlope);

                    // 縦断勾配タイプ
                    if (isValidVSlope)
                    {
                        vtclSlopeType = getVtclSlopeType(nMaxVSlope, isVEndHigher);
                        strVSlopeRank = getVSlopeRank(nMaxVSlope);
                    }
                }

                // 横断勾配計測
                bool isValidHSlope = false;
                int nMaxHSlope = 99;
                Boost3DPointHash maxHSlopePos;
                if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3
                    && CEpsUtil::GreaterEqual(m_dLod3Detail, 3.2)
                    && isValidWidth && CEpsUtil::Greater(dMinWidth, dMinWidthThForHSlope))
                {
                    isValidHSlope = calcCrossGradient(
                        route, roadPtr, frnPtr, bridgePtr,
                        dFootpathHSlopeInterval, nMaxHSlope, maxHSlopePos);
                }

                // ランク区分
                std::string strRank = strWidthRank + strVSlopeRank + "X";   // 段差はX:不明で固定

                // 経路の構造
                ROUTE_STRUCTURE_TYPE rsType = getRouteStructureType(roadPtr, frnPtr, bridgePtr);

                Link link(strLinkId, route, dLength, isValidWidth, isRefMinWidth, dMinWidth,
                    minWidthPos, widthType, strCreateDate, nodeIds[0], nodeIds[1],
                    strName, strRank, brailleType, rsType, isValidVSlope, vtclSlopeType,
                    nMaxVSlope, maxVSlopePos, nAveVSlope,
                    isValidHSlope, nMaxHSlope, maxHSlopePos);
                {
                    std::lock_guard<std::mutex> lock(m_linkMutex);
                    links.push_back(link);
                    geoms.push_back(link.geom);
                }
                isAdd = true;
            }

            // 探索済み経路の登録
            searchedDesc.insert(std::pair<BoostVertexDesc, BoostVertexDesc>(from, to));
            searchedDesc.insert(std::pair<BoostVertexDesc, BoostVertexDesc>(to, from));
        }
    }

    return isAdd;
}


// ノード情報のフィールド情報作成
std::vector<CGISFileAttribute::AttributeFieldData> CNetwork::createNodeFields(
    const int nMaxLinkNum,
    const NETWORK_DATA_TYPE dataType)
{
    std::vector<CGISFileAttribute::AttributeFieldData> fields;

    // ID
    // 平面直角座標系の系番号2桁
    // 符号1桁(正:0, 負:1) + 座標の整数部6桁 (xyz分)
    // 小数部は6桁(連番用)
    // 小数点含め, 計30文字
    CGISFileAttribute::AttributeFieldData fieldId;
    fieldId.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_STRING;
    fieldId.strName = "node_id";
    fieldId.nWidth = m_nNodeIdDigit;
    fields.push_back(fieldId);

    // 経緯度は小数点以下15桁
    // 緯度
    CGISFileAttribute::AttributeFieldData fieldLat;
    fieldLat.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_DOUBLE;
    fieldLat.strName = "lat";
    fieldLat.nWidth = m_nLanLotDigit;
    fieldLat.nDecimals = m_nLanLotDecimal;
    fields.push_back(fieldLat);

    // 経度
    CGISFileAttribute::AttributeFieldData fieldLon;
    fieldLon.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_DOUBLE;
    fieldLon.strName = "lon";
    fieldLon.nWidth = m_nLanLotDigit;
    fieldLon.nDecimals = m_nLanLotDecimal;
    fields.push_back(fieldLon);

    if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
    {
        // 標高値
        CGISFileAttribute::AttributeFieldData fieldEve;
        fieldEve.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_DOUBLE;
        fieldEve.strName = "elevation";
        fieldEve.nWidth = 10;
        fieldEve.nDecimals = 1;
        fields.push_back(fieldEve);
    }

    if (dataType == NETWORK_DATA_TYPE::FOOTPATH)
    {
        // 施設内外区分
        CGISFileAttribute::AttributeFieldData fieldInOut;
        fieldInOut.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_INT;
        fieldInOut.strName = "in_out";
        fieldInOut.nWidth = 1;
        fields.push_back(fieldInOut);
    }

    for (int i = 1; i <= nMaxLinkNum; i++)
    {
        // 接続リンク
        CGISFileAttribute::AttributeFieldData field;
        field.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_STRING;
        field.strName = (boost::format("link%d_id") % i).str();
        field.nWidth = m_nLinkIdDigit;
        fields.push_back(field);
    }

    return fields;
}

// リンク情報のフィールド情報作成
std::vector<CGISFileAttribute::AttributeFieldData> CNetwork::createLinkFields(
    const NETWORK_DATA_TYPE dataType)
{
    std::vector<CGISFileAttribute::AttributeFieldData> fields;

    // ID
    CGISFileAttribute::AttributeFieldData fieldId;
    fieldId.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_STRING;
    fieldId.strName = "link_id";
    fieldId.nWidth = m_nLinkIdDigit;
    fields.push_back(fieldId);

    // ノードID
    // 平面直角座標系の系番号2桁
    // 符号1桁(正:0, 負:1) + 座標の整数部6桁 (xyz分)
    // 小数部は6桁(連番用)
    // 小数点含め, 計30文字
    // 開始ノードID
    CGISFileAttribute::AttributeFieldData fieldStartId;
    fieldStartId.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_STRING;
    fieldStartId.strName = "start_id";
    fieldStartId.nWidth = m_nNodeIdDigit;
    fields.push_back(fieldStartId);

    // 終点ノードID
    CGISFileAttribute::AttributeFieldData fieldEndId;
    fieldEndId.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_STRING;
    fieldEndId.strName = "end_id";
    fieldEndId.nWidth = m_nNodeIdDigit;
    fields.push_back(fieldEndId);

    // リンク延長と幅員は小数点以下第3位までの精度とする
    // リンク延長
    CGISFileAttribute::AttributeFieldData fieldDist;
    fieldDist.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_DOUBLE;
    fieldDist.strName = "distance";
    fieldDist.nWidth = 10;
    fieldDist.nDecimals = 1;
    fields.push_back(fieldDist);

    if (dataType == NETWORK_DATA_TYPE::FOOTPATH)
    {
        // ランク区分
        CGISFileAttribute::AttributeFieldData fieldRank;
        fieldRank.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_STRING;
        fieldRank.strName = "rank";
        fieldRank.nWidth = 3;
        fields.push_back(fieldRank);
    }

    // リンク作成・更新日
    CGISFileAttribute::AttributeFieldData fieldDate;
    fieldDate.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_STRING;
    fieldDate.strName = "maint_date";
    fieldDate.nWidth = 10;  // YYYY-MM-DD
    fields.push_back(fieldDate);

    if (dataType == NETWORK_DATA_TYPE::FOOTPATH)
    {
        // 経路構造
        CGISFileAttribute::AttributeFieldData fieldRtStruct;
        fieldRtStruct.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_INT;
        fieldRtStruct.strName = "rt_struct";
        fieldRtStruct.nWidth = 2;
        fields.push_back(fieldRtStruct);

        // 幅員タイプ
        CGISFileAttribute::AttributeFieldData fieldWidth;
        fieldWidth.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_INT;
        fieldWidth.strName = "width";
        fieldWidth.nWidth = 2;
        fields.push_back(fieldWidth);
    }

    if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
    {
        // 縦断勾配タイプ
        CGISFileAttribute::AttributeFieldData fieldVtclSlope;
        fieldVtclSlope.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_INT;
        fieldVtclSlope.strName = "vtcl_slope";
        fieldVtclSlope.nWidth = 2;
        fields.push_back(fieldVtclSlope);
    }

    if (dataType == NETWORK_DATA_TYPE::FOOTPATH)
    {
        // 視覚障害者用誘導ブロックの有無
        CGISFileAttribute::AttributeFieldData fieldBrailleTile;
        fieldBrailleTile.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_INT;
        fieldBrailleTile.strName = "brail_tile";
        fieldBrailleTile.nWidth = 2;
        fields.push_back(fieldBrailleTile);
    }

    // 最小幅員
    CGISFileAttribute::AttributeFieldData fieldWMin;
    fieldWMin.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_DOUBLE;
    fieldWMin.strName = "w_min";
    fieldWMin.nWidth = 10;
    fieldWMin.nDecimals = 1;
    fields.push_back(fieldWMin);
    if (dataType == NETWORK_DATA_TYPE::FOOTPATH)
    {
        // 最小幅員緯度経度
        CGISFileAttribute::AttributeFieldData fieldWMinLat, fieldWMinLon;
        fieldWMinLat.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_DOUBLE;
        fieldWMinLat.strName = "w_min_lat";
        fieldWMinLat.nWidth = m_nLanLotDigit;
        fieldWMinLat.nDecimals = m_nLanLotDecimal;
        fields.push_back(fieldWMinLat);
        fieldWMinLon.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_DOUBLE;
        fieldWMinLon.strName = "w_min_lon";
        fieldWMinLon.nWidth = m_nLanLotDigit;
        fieldWMinLon.nDecimals = m_nLanLotDecimal;
        fields.push_back(fieldWMinLon);
        // 最小幅員参考値フラグ
        CGISFileAttribute::AttributeFieldData fieldRefWMin;
        fieldRefWMin.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_INT;
        fieldRefWMin.strName = "ref_w_min";
        fieldRefWMin.nWidth = 2;
        fields.push_back(fieldRefWMin);
    }
    else if (dataType == NETWORK_DATA_TYPE::ROADWAY)
    {
        // 最小幅員値の有効/無効フラグ
        CGISFileAttribute::AttributeFieldData fieldIsValidValue;
        fieldIsValidValue.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_INT;
        fieldIsValidValue.strName = "is_w_min";
        fieldIsValidValue.nWidth = 2;
        fields.push_back(fieldIsValidValue);
    }

    if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
    {
        // 縦断勾配
        CGISFileAttribute::AttributeFieldData fieldVSlopeMax;
        fieldVSlopeMax.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_INT;
        fieldVSlopeMax.strName = "vSlope_max";
        fieldVSlopeMax.nWidth = 3;
        fields.push_back(fieldVSlopeMax);
        if (dataType == NETWORK_DATA_TYPE::FOOTPATH)
        {
            CGISFileAttribute::AttributeFieldData fieldVSlopeLat, fieldVSlopeLon;
            fieldVSlopeLat.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_DOUBLE;
            fieldVSlopeLat.strName = "vSlope_lat";
            fieldVSlopeLat.nWidth = m_nLanLotDigit;
            fieldVSlopeLat.nDecimals = m_nLanLotDecimal;
            fields.push_back(fieldVSlopeLat);
            fieldVSlopeLon.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_DOUBLE;
            fieldVSlopeLon.strName = "vSlope_lon";
            fieldVSlopeLon.nWidth = m_nLanLotDigit;
            fieldVSlopeLon.nDecimals = m_nLanLotDecimal;
            fields.push_back(fieldVSlopeLon);

        }
        CGISFileAttribute::AttributeFieldData fieldVSlopeAve;
        fieldVSlopeAve.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_INT;
        fieldVSlopeAve.strName = "vSlope_ave";
        fieldVSlopeAve.nWidth = 3;
        fields.push_back(fieldVSlopeAve);

        if (dataType == NETWORK_DATA_TYPE::ROADWAY)
        {
            // 縦断勾配の有効/無効フラグ
            CGISFileAttribute::AttributeFieldData fieldIsValidValue;
            fieldIsValidValue.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_INT;
            fieldIsValidValue.strName = "is_vSlope";
            fieldIsValidValue.nWidth = 2;
            fields.push_back(fieldIsValidValue);
        }
    }

    if (dataType == NETWORK_DATA_TYPE::ROADWAY)
    {
        // 道路の区分
        CGISFileAttribute::AttributeFieldData fieldType;
        fieldType.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_STRING;
        fieldType.strName = "type";
        fieldType.nWidth = 64;
        fields.push_back(fieldType);
    }

    if (dataType == NETWORK_DATA_TYPE::FOOTPATH
        && CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3
        && CEpsUtil::GreaterEqual(m_dLod3Detail, 3.2))
    {
        // 横断勾配
        CGISFileAttribute::AttributeFieldData fieldHSlopeMax, fieldHSlopeLat, fieldHSlopeLon;
        fieldHSlopeMax.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_INT;
        fieldHSlopeMax.strName = "hSlope_max";
        fieldHSlopeMax.nWidth = 3;
        fields.push_back(fieldHSlopeMax);
        fieldHSlopeLat.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_DOUBLE;
        fieldHSlopeLat.strName = "hSlope_lat";
        fieldHSlopeLat.nWidth = m_nLanLotDigit;
        fieldHSlopeLat.nDecimals = m_nLanLotDecimal;
        fields.push_back(fieldHSlopeLat);
        fieldHSlopeLon.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_DOUBLE;
        fieldHSlopeLon.strName = "hSlope_lon";
        fieldHSlopeLon.nWidth = m_nLanLotDigit;
        fieldHSlopeLon.nDecimals = m_nLanLotDecimal;
        fields.push_back(fieldHSlopeLon);
    }

    // 通り名、路線名
    CGISFileAttribute::AttributeFieldData fieldRtName;
    fieldRtName.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_STRING;
    fieldRtName.strName = "route_name";
    fieldRtName.nWidth = 128;
    fields.push_back(fieldRtName);

    return fields;
}

// ノード情報の幾何形状と属性情報の作成
void CNetwork::createNodeData(
    const std::vector<Node> &nodes,
    const int nMaxLinkNum,
    const NETWORK_DATA_TYPE dataType,
    const int nJPZone,
    Boost3DMultiPointHashs &pts,
    std::vector<CGISFileAttribute::AttributeDataRecord> &attrRecords)
{
    pts.clear();
    attrRecords.clear();

    int nId = 0;
    for (const auto node : nodes)
    {
        // 幾何形状
        double dLat, dLon;
        CGeoUtil::XYToLatLon(nJPZone, node.pt.y(), node.pt.x(), dLat, dLon);    // x:東西, y:南北
        pts.push_back(Boost3DPointHash(dLon, dLat, node.pt.z()));

        // 属性
        CGISFileAttribute::AttributeDataRecord record;
        record.nShapeId = nId;

        // データの格納順は属性フィールド順と揃えているため崩さないこと
        // ID
        record.vecAttribute.push_back(CGISFileAttribute::AttributeData(node.strId));

        // 緯度
        record.vecAttribute.push_back(CGISFileAttribute::AttributeData(dLat));

        // 経度
        record.vecAttribute.push_back(CGISFileAttribute::AttributeData(dLon));

        if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
        {
            // 標高値
            record.vecAttribute.push_back(CGISFileAttribute::AttributeData(CUtil::RoundN(node.pt.z(), 1)));
        }

        if (dataType == NETWORK_DATA_TYPE::FOOTPATH)
        {
            // 施設内外区分(施設外で固定)
            record.vecAttribute.push_back(CGISFileAttribute::AttributeData(1));
        }

        // 接続リンクID
        int nNum = 0;
        for (const std::string &strLinkId : node.edges)
        {
            record.vecAttribute.push_back(CGISFileAttribute::AttributeData(strLinkId));
            nNum++;
        }
        for (; nNum < nMaxLinkNum; nNum++)
            record.vecAttribute.push_back(CGISFileAttribute::AttributeData());  // NULL値で埋める

        attrRecords.push_back(record);
        nId++;
    }
}

// リンク情報の幾何形状と属性情報の作成
void CNetwork::createLinkData(
    const std::vector<Link> &links,
    const NETWORK_DATA_TYPE dataType,
    const int nJPZone,
    Boost3DHashMultiLines &polylines,
    std::vector<CGISFileAttribute::AttributeDataRecord> &attrRecords)
{
    polylines.clear();
    attrRecords.clear();
    double dLat, dLon;

    int nId = 0;
    for (const auto link : links)
    {
        // 幾何形状
        Boost3DHashPolyline polyline;
        for (const auto &pt : link.geom)
        {
            CGeoUtil::XYToLatLon(nJPZone, pt.y(), pt.x(), dLat, dLon);    // x:東西, y:南北
            polyline.push_back(Boost3DPointHash(dLon, dLat, pt.z()));
        }
        polylines.push_back(polyline);

        // 属性
        CGISFileAttribute::AttributeDataRecord record;
        record.nShapeId = nId;

        // データの格納順は属性フィールド順と揃えているため崩さないこと
        // ID
        record.vecAttribute.push_back(CGISFileAttribute::AttributeData(link.strId));

        // 開始ノードID
        record.vecAttribute.push_back(CGISFileAttribute::AttributeData(link.strStartId));

        // 終点ノードID
        record.vecAttribute.push_back(CGISFileAttribute::AttributeData(link.strEndId));

        // リンク延長
        record.vecAttribute.push_back(CGISFileAttribute::AttributeData(link.dLength));

        if (dataType == NETWORK_DATA_TYPE::FOOTPATH)
        {
            // ランク区分
            record.vecAttribute.push_back(CGISFileAttribute::AttributeData(link.strRank));
        }

        // リンク作成・更新日
        record.vecAttribute.push_back(CGISFileAttribute::AttributeData(link.strCreateDate));

        if (dataType == NETWORK_DATA_TYPE::FOOTPATH)
        {
            // 経路構造
            record.vecAttribute.push_back(
                CGISFileAttribute::AttributeData(
                    static_cast<std::underlying_type<ROUTE_STRUCTURE_TYPE>::type>(link.rtStruct)));

            // 幅員タイプ
            record.vecAttribute.push_back(
                CGISFileAttribute::AttributeData(
                    static_cast<std::underlying_type<WIDTH_TYPE>::type>(link.width)));
        }

        if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
        {
            // 縦断勾配タイプ
            record.vecAttribute.push_back(
                CGISFileAttribute::AttributeData(
                    static_cast<std::underlying_type<VTCL_SLOPE_TYPE>::type>(link.vtclSlope)));
        }

        if (dataType == NETWORK_DATA_TYPE::FOOTPATH)
        {
            // 視覚障害者用誘導ブロックの有無
            record.vecAttribute.push_back(
                CGISFileAttribute::AttributeData(
                    static_cast<std::underlying_type<BRAILLE_TILE_TYPE>::type>(link.brailTile)));
        }

        // 最小幅員
        record.vecAttribute.push_back(CGISFileAttribute::AttributeData(link.dMinWidth));
        if (dataType == NETWORK_DATA_TYPE::FOOTPATH)
        {
            CGeoUtil::XYToLatLon(nJPZone, link.minWidthPos.y(), link.minWidthPos.x(), dLat, dLon);    // x:東西, y:南北
            record.vecAttribute.push_back(CGISFileAttribute::AttributeData(dLat));
            record.vecAttribute.push_back(CGISFileAttribute::AttributeData(dLon));
            int nRef = (link.isRefMinWidth) ? 1 : 0;    // 参考値フラグ
            record.vecAttribute.push_back(CGISFileAttribute::AttributeData(nRef));
        }
        else if (dataType == NETWORK_DATA_TYPE::ROADWAY)
        {
            // 最小幅員値の有効/無効フラグ
            IS_VALID validType = (link.isValidWidth) ? IS_VALID::VALID_VALUE : IS_VALID::INVALID_VALUE;
            record.vecAttribute.push_back(CGISFileAttribute::AttributeData(
                static_cast<std::underlying_type<IS_VALID>::type>(validType)));
        }

        if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
        {
            // 縦断勾配
            record.vecAttribute.push_back(CGISFileAttribute::AttributeData(link.nMaxVSlope));
            if (dataType == NETWORK_DATA_TYPE::FOOTPATH)
            {
                CGeoUtil::XYToLatLon(nJPZone, link.vSlopePos.y(), link.vSlopePos.x(), dLat, dLon);    // x:東西, y:南北
                record.vecAttribute.push_back(CGISFileAttribute::AttributeData(dLat));
                record.vecAttribute.push_back(CGISFileAttribute::AttributeData(dLon));
            }
            record.vecAttribute.push_back(CGISFileAttribute::AttributeData(link.nAveVSlope));

            if (dataType == NETWORK_DATA_TYPE::ROADWAY)
            {
                // 最小幅員値の有効/無効フラグ
                IS_VALID validType = (link.isValidVSlope) ? IS_VALID::VALID_VALUE : IS_VALID::INVALID_VALUE;
                record.vecAttribute.push_back(CGISFileAttribute::AttributeData(
                    static_cast<std::underlying_type<IS_VALID>::type>(validType)));
            }
        }

        if (dataType == NETWORK_DATA_TYPE::ROADWAY)
        {
            // 道路の区分
            record.vecAttribute.push_back(CGISFileAttribute::AttributeData(link.strType));
        }
        if (dataType == NETWORK_DATA_TYPE::FOOTPATH
            && CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3
            && CEpsUtil::GreaterEqual(m_dLod3Detail, 3.2))
        {
            // 横断勾配
            CGeoUtil::XYToLatLon(nJPZone, link.hSlopePos.y(), link.hSlopePos.x(), dLat, dLon);    // x:東西, y:南北
            record.vecAttribute.push_back(CGISFileAttribute::AttributeData(link.nMaxHSlope));
            record.vecAttribute.push_back(CGISFileAttribute::AttributeData(dLat));
            record.vecAttribute.push_back(CGISFileAttribute::AttributeData(dLon));
        }

        // 通り名、路線名
        record.vecAttribute.push_back(CGISFileAttribute::AttributeData(link.strName));
        attrRecords.push_back(record);
        nId++;
    }
}

// 縦断勾配(コード値)の取得
VTCL_SLOPE_TYPE CNetwork::getVtclSlopeType(const int nVtclSlope, const bool isVEndHigher)
{
    VTCL_SLOPE_TYPE type;

    if (nVtclSlope == 0)
    {
        type = VTCL_SLOPE_TYPE::ZERO;
    }
    else if (nVtclSlope > 0 && nVtclSlope <= 5)
    {
        type = VTCL_SLOPE_TYPE::GREATER_THAN_0_BUT_LESS_THAN_OR_EQUAL_5;
    }
    else if (nVtclSlope > 5 && nVtclSlope <= 8)
    {
        if (isVEndHigher)
        {
            type = VTCL_SLOPE_TYPE::GREATER_THAN_5_BUT_LESS_THAN_OR_EQUAL_8_END_POINT_HIGHER;
        }
        else
        {
            type = VTCL_SLOPE_TYPE::GREATER_THAN_5_BUT_LESS_THAN_OR_EQUAL_8_START_POINT_HIGHER;
        }
    }
    else if (nVtclSlope > 8 && nVtclSlope <= 18)
    {
        if (isVEndHigher)
        {
            type = VTCL_SLOPE_TYPE::GREATER_THAN_8_BUT_LESS_THAN_OR_EQUAL_18_END_POINT_HIGHER;
        }
        else
        {
            type = VTCL_SLOPE_TYPE::GREATER_THAN_8_BUT_LESS_THAN_OR_EQUAL_18_START_POINT_HIGHER;
        }
    }
    else if (nVtclSlope > 18)
    {
        if (isVEndHigher)
        {
            type = VTCL_SLOPE_TYPE::GREATER_THAN_18_END_POINT_HIGHER;
        }
        else
        {
            type = VTCL_SLOPE_TYPE::GREATER_THAN_18_START_POINT_HIGHER;
        }
    }
    else
    {
        type = VTCL_SLOPE_TYPE::UNKNOWN;
    }

    return type;
}

// 歩道用幅員(コード値)の取得
WIDTH_TYPE CNetwork::getWidthType(const double dWidth)
{
    WIDTH_TYPE type;

    if (CEpsUtil::Greater(dWidth, 0) && CEpsUtil::Less(dWidth, 1.0))
    {
        type = WIDTH_TYPE::LESS_THAN_1M;
    }
    else if (CEpsUtil::GreaterEqual(dWidth, 1.0) && CEpsUtil::Less(dWidth, 2.0))
    {
        type = WIDTH_TYPE::MORE_THAN_1M_BUT_LESS_THAN_2M;
    }
    else if (CEpsUtil::GreaterEqual(dWidth, 2.0) && CEpsUtil::Less(dWidth, 3.0))
    {
        type = WIDTH_TYPE::MORE_THAN_2M_BUT_LESS_THAN_3M;
    }
    else if (CEpsUtil::GreaterEqual(dWidth, 3.0))
    {
        type = WIDTH_TYPE::MORE_THAN_3M;
    }
    else
    {
        type = WIDTH_TYPE::UNKNOWN;
    }

    return type;
}

// 経路構造の取得
ROUTE_STRUCTURE_TYPE CNetwork::getRouteStructureType(
    const std::shared_ptr<CTranRoadData> &roadPtr,
    const std::shared_ptr<CFurnitureData> &frnPtr,
    const std::shared_ptr<CBridgeData> &bridgePtr)
{
    ROUTE_STRUCTURE_TYPE type;
    if (roadPtr != nullptr)
        type = ROUTE_STRUCTURE_TYPE::SEPARATED_DRIVEWAY_AND_SIDEWALK;
    else if (frnPtr != nullptr)
        type = ROUTE_STRUCTURE_TYPE::PEEDESTRIAN_CROSSING;
    else if (bridgePtr != nullptr)
        type = ROUTE_STRUCTURE_TYPE::PEDESTRIAN_BRIDGE;
    else
        type = ROUTE_STRUCTURE_TYPE::UNKNOWN;

    return type;
}

 // 幅員ランクの取得
std::string CNetwork::getWidthRank(const double dWidth)
{
    // Bなし
    // Cはモビリティ通行可能な1未満だが、モビリティの通行可/不可が判断できない
    std::string strRank;

    if (CEpsUtil::GreaterEqual(dWidth, 2.0))
        strRank = "S";
    else if (CEpsUtil::GreaterEqual(dWidth, 1.0) && CEpsUtil::Less(dWidth, 2.0))
        strRank = "A";
    else if (CEpsUtil::Less(dWidth, 1.0) && CEpsUtil::Greater(dWidth, 0))
        strRank = "Z";
    else
        strRank = "X";  // 不明
    return strRank;
}

//  縦断勾配ランクの取得
std::string CNetwork::getVSlopeRank(const int nVSlope)
{
    std::string strRank;

    if (nVSlope == 0)
        strRank = "S";
    else if (nVSlope > 0 && nVSlope <= 5)
        strRank = "A";
    else if (nVSlope > 5 && nVSlope <= 8)
        strRank = "B";
    else if (nVSlope > 8 && nVSlope <= 18)
        strRank = "C";
    else if (nVSlope > 18)
        strRank = "Z";
    else
        strRank = "X";  // 不明
    return strRank;

}

// 幅員の計測
bool CNetwork::measureWidth(
    const Boost3DHashPolyline &line,
    const std::shared_ptr<CTranRoadData> &roadPtr,
    const NETWORK_DATA_TYPE type,
    const double dInterval,
    double &dMinWidth,
    Boost3DPointHash &pt,
    bool &isRefMinWidth)
{
    dMinWidth = 0;
    isRefMinWidth = false;

    std::vector<std::pair<CVector2D, CVector2D>> edgeVecPairList; // エッジの始点と方向ベクトルのペア
    if (type == NETWORK_DATA_TYPE::ROADWAY)
    {
        // 道路の場合
        // 道路縁をセグメント毎に始点と方向ベクトルを保存
        for (const auto &edgePair : roadPtr->edgePairList)
        {
            for (auto it = edgePair.first.cbegin(); it < edgePair.first.cend() - 1; it++)
            {
                CVector2D startPoint = CBoostGeoUtil::ToCVector2D(*it);
                CVector2D endPoint = CBoostGeoUtil::ToCVector2D(*(it + 1));
                CVector2D edgeVec = endPoint - startPoint;
                edgeVecPairList.emplace_back(std::make_pair(edgeVec, startPoint));
            }

            for (auto it = edgePair.second.cbegin(); it < edgePair.second.cend() - 1; it++)
            {
                CVector2D startPoint = CBoostGeoUtil::ToCVector2D(*it);
                CVector2D endPoint = CBoostGeoUtil::ToCVector2D(*(it + 1));
                CVector2D edgeVec = endPoint - startPoint;
                edgeVecPairList.emplace_back(std::make_pair(edgeVec, startPoint));
            }
        }
    }
    else
    {
        // 歩道の場合
        // 中心線に該当する歩道ポリゴンの探索
        Boost3DHashPolygon targetPolygon;
        if (CTranRoadDataUtil::SearchFootpathPolygon(*roadPtr, line, targetPolygon))
        {
            // 抽出済みのエッジ線(中心線作成用)に注目歩道ポリゴンのエッジ線が存在するか確認する
            for (const auto &edgePair : roadPtr->m_footpath.edgePairList)
            {
                if (!CBoostGeoUtil::Disjoint(targetPolygon, edgePair.first)
                    && !CBoostGeoUtil::Disjoint(targetPolygon, edgePair.second))
                {
                    for (auto it = edgePair.first.cbegin(); it < edgePair.first.cend() - 1; it++)
                    {
                        CVector2D startPoint = CBoostGeoUtil::ToCVector2D(*it);
                        CVector2D endPoint = CBoostGeoUtil::ToCVector2D(*(it + 1));
                        CVector2D edgeVec = endPoint - startPoint;
                        edgeVecPairList.emplace_back(std::make_pair(edgeVec, startPoint));
                    }

                    for (auto it = edgePair.second.cbegin(); it < edgePair.second.cend() - 1; it++)
                    {
                        CVector2D startPoint = CBoostGeoUtil::ToCVector2D(*it);
                        CVector2D endPoint = CBoostGeoUtil::ToCVector2D(*(it + 1));
                        CVector2D edgeVec = endPoint - startPoint;
                        edgeVecPairList.emplace_back(std::make_pair(edgeVec, startPoint));
                    }
                }
            }

            if (edgeVecPairList.size() == 0)
            {
                isRefMinWidth = true;   // 歩道の輪郭線から算出する場合は参考値フラグをON
                // 歩道の輪郭線の始点と方向ベクトルを保存
                for (auto it = targetPolygon.outer().cbegin();
                    it < targetPolygon.outer().cend() - 1; it++)
                {
                    CVector2D startPoint = CBoostGeoUtil::ToCVector2D(*it);
                    CVector2D endPoint = CBoostGeoUtil::ToCVector2D(*(it + 1));
                    CVector2D edgeVec = endPoint - startPoint;
                    edgeVecPairList.emplace_back(std::make_pair(edgeVec, startPoint));
                }
            }
        }
    }

    if (edgeVecPairList.size() > 0)
    {
        auto sampledCenterLine = CBoostGeoUtil::Sampling(line, dInterval);

        // 中心線のセグメント毎に処理
        for (auto it = sampledCenterLine.cbegin();
            it < sampledCenterLine.cend() - 1; it++)
        {
            /** △     ：firstPoint
             *  ▲     ：secondPoint
             *  〇     ：rightCrossPoint
             *  ●     ：leftCrossPoint
             *  △->〇 ：rightVec
             *  △->● ：leftVec
             *  │     ：centerLine
             *  ┃     ：edge
             *
             * ┃ 　　　│　　　 ┃
             * ┃ 　　　▲　　　 ┃
             * ┃ 　　　│　　　 ┃
             * ┃ 　　　│　　　 ┃
             * ┃ 　　　│　　　 ┃
             * ┃ 　　　│　　　 ┃
             * ┃ 　　　├┐　　 ┃
             * ●<- - - △┴ - ->〇
             * ┃ 　　　　　　　 ┃
             * ┃<- roadWidth - >┃
             */

             ///
             /// 各情報を取得
             ///

            CVector2D firstPoint = CBoostGeoUtil::ToCVector2D(*it);
            CVector2D secondPoint = CBoostGeoUtil::ToCVector2D(*(it + 1));

            CVector2D inputVec = secondPoint - firstPoint;
            CVector2D rightVec;
            CVector2D leftVec;

            // 注目点(firstPoint)から左右に伸びる垂線ベクトルを求める
            if (CGeoUtil::GetVerticalVec(inputVec, rightVec) == false)
            {
                continue;
            }

            leftVec = -1 * rightVec;

            ///
            /// 左右の交点算出
            ///
            CVector2D rightCrossPoint;
            CVector2D leftCrossPoint;
            double rightCrossLength = LDBL_MAX;
            double leftCrossLength = LDBL_MAX;
            bool isRightCrossPoint = false;
            bool isLeftCrossPoint = false;

            // 歩道において歩道部ポリゴンから左右のエッジ候補作成した場合、
            // 隣接道路と重畳するエッジも残っているため、誤って交点を算出しないように
            // 注目中心線セグメントの中点から左右に垂線をおろす
            CVector2D centerPoint = inputVec * 0.5 + firstPoint;

            // 保存しなおしたセグメント毎に処理
            for (auto &edgeVecPair : edgeVecPairList)
            {
                bool isCross = false;
                CVector2D crossPointTmp;
                bool isOnLine1 = false;
                bool isOnLine2 = false;
                double t = 0.0;
                double s = 0.0;

                // 右側交点算出
                isCross = CGeoUtil::GetCrossPos(
                    rightVec, centerPoint,
                    edgeVecPair.first, edgeVecPair.second,
                    crossPointTmp, isOnLine1, isOnLine2, t, s);

                // 交点の取得成功
                // かつ、エッジセグメント上に交点がある
                // かつ、垂直方向に正の向きに交点がある
                // かつ、交点までの距離がより短い
                // かつ、交点までの距離が0ではない
                if (isCross == true
                    && isOnLine2 == true
                    && CEpsUtil::Greater(t, 0)
                    && CEpsUtil::Greater(rightCrossLength, t)
                    && !CEpsUtil::Zero(CUtil::RoundN(t, 3)))
                {

                    // 注目点(firstPoint)から一番近い交点を保存
                    rightCrossLength = t;
                    rightCrossPoint = crossPointTmp;
                    isRightCrossPoint = true;
                }

                // 左側交点算出
                isCross = CGeoUtil::GetCrossPos(
                    leftVec, centerPoint,
                    edgeVecPair.first, edgeVecPair.second,
                    crossPointTmp, isOnLine1, isOnLine2, t, s);

                // 交点の取得成功
                // かつ、エッジセグメント上に交点がある
                // かつ、垂直方向に正の向きに交点がある
                // かつ、交点までの距離がより短い
                // かつ、交点までの距離が0ではない
                if (isCross == true
                    && isOnLine2 == true
                    && CEpsUtil::Greater(t, 0)
                    && CEpsUtil::Greater(leftCrossLength, t)
                    && !CEpsUtil::Zero(CUtil::RoundN(t, 3)))
                {
                    // 注目点(firstPoint)から一番近い交点を保存
                    leftCrossLength = t;
                    leftCrossPoint = crossPointTmp;
                    isLeftCrossPoint = true;
                }
            }

            // 左右の交点が算出できなかった場合はスキップ
            if (isRightCrossPoint == false || isLeftCrossPoint == false)
            {
                continue;
            }

            // このセグメントの幅員を測定する
            double segmentWidth = rightCrossPoint.Distance(leftCrossPoint);

            // 中心線の幅員より小さければ、
            // この中心線の最小幅員と計測地点を更新する
            if (CEpsUtil::LessEqual(dMinWidth, 0.0) || CEpsUtil::Greater(dMinWidth, segmentWidth))
            {
                dMinWidth = segmentWidth;
                pt.x(centerPoint.x);
                pt.y(centerPoint.y);
            }
        }
    }

    return CEpsUtil::Greater(dMinWidth, 0);
}

// 点字ブロックの有無の設定
BRAILLE_TILE_TYPE CNetwork::checkBrailleTile(
    const Boost3DHashPolyline &line,
    const std::shared_ptr<CTranRoadData> &roadPtr,
    const std::shared_ptr<CFurnitureData> &crossingPtr,
    const double dDistTh)
{
    BRAILLE_TILE_TYPE brailleType = BRAILLE_TILE_TYPE::UNKNOWN;
    const double dFootpathSamplingInterval = 1.0;   // 歩道のサンプリング間隔
    const double dCrossingSamplingInterval = 0.1;   // 横断歩道のサンプリング間隔

    // 中心線をサンプリングし、サンプリング点の近傍に点字ブロックが存在するか確認する
    // 歩道中心線
    if (roadPtr != nullptr
        && roadPtr->m_footpath.centerLineList.size() > 0)
    {
        // 中心線と衝突する歩道ポリゴンの探索
        Boost3DHashPolygon targetPolygon;
        if (CTranRoadDataUtil::SearchFootpathPolygon(*roadPtr, line, targetPolygon))
        {
            // 中心線をサンプリング
            Boost3DHashPolyline samplingLine = CBoostGeoUtil::Sampling(line, dFootpathSamplingInterval);
            size_t cnt = 0;
            for (const auto &pt : samplingLine)
            {
                std::vector<BrailleTileTuple> results;
                m_brailleTileRTree.query(bg::index::nearest(pt, 1), std::back_inserter(results));
                if (results.size() > 0)
                {
                    auto [brailleTilePt, targetFrmPtr, polygonPtr] = results.front();
                    double dDist = bg::distance(brailleTilePt, pt);
                    // 最近傍点字ブロックと歩道ポリゴンの衝突確認
                    if (!CBoostGeoUtil::Disjoint(*polygonPtr, targetPolygon)
                        && CEpsUtil::LessEqual(dDist, dDistTh))
                    {
                        cnt++;
                    }
                }
            }
            if (cnt >= samplingLine.size() / 2)
            {
                brailleType = BRAILLE_TILE_TYPE::EXIST;
            }
            else
            {
                brailleType = BRAILLE_TILE_TYPE::DOES_NOT_EXIST;
            }
        }
    }

    // 横断歩道
    if (crossingPtr != nullptr
        && crossingPtr->m_pedestrianCrossingData.m_bUse)
    {
        if (!bg::is_empty(crossingPtr->m_pedestrianCrossingData.m_centerLineData.centerLine))
        {
            // 中心線をサンプリング
            Boost3DHashPolyline samplingLine = CBoostGeoUtil::Sampling(
                crossingPtr->m_pedestrianCrossingData.m_centerLineData.centerLine,
                dCrossingSamplingInterval);
            size_t cnt = 0;
            for (const auto &pt : samplingLine)
            {
                std::vector<BrailleTileTuple> results;
                m_brailleTileRTree.query(bg::index::nearest(pt, 1), std::back_inserter(results));
                if (results.size() > 0)
                {
                    auto [brailTilePt, targetFrmPtr, polygonPtr] = results.front();
                    double dDist = bg::distance(brailTilePt, pt);
                    // 最近傍点字ブロックと歩道ポリゴンの衝突確認
                    if (!CBoostGeoUtil::Disjoint(*polygonPtr, crossingPtr->m_pedestrianCrossingData.m_mbr)
                        && CEpsUtil::LessEqual(dDist, dDistTh))
                    {
                        cnt++;
                    }
                }
            }

            if (cnt >= samplingLine.size() / 2)
            {
                brailleType = BRAILLE_TILE_TYPE::EXIST;
            }
            else
            {
                brailleType = BRAILLE_TILE_TYPE::DOES_NOT_EXIST;
            }
        }
    }
    return brailleType;
}

// 勾配計測(道路、横断歩道用)
bool CNetwork::calcGradient(
    const Boost3DHashPolyline &line,
    const std::shared_ptr<CTranRoadData> &roadPtr,
    const std::shared_ptr<CFurnitureData> &crossingPtr,
    const bool bSamplingFlag,
    const double dInterval,
    int &nMaxVSlope,
    double &dMaxVSlope,
    bool &isEndHigher,
    Boost3DPointHash &vSlopePos,
    int &nAveVSlope)
{
    bool bRet = false;
    double dMax = 0;            // 最大勾配
    int n = 0;                  // 計測数
    double dHorizontalDist = 0; // 平均勾配用

    // 中心線を水平距離でサンプリング
    Boost3DHashPolyline samplingLine = bSamplingFlag ? CBoostGeoUtil::Sampling2D(line, dInterval) : line;

    // サンプリング点の高さを取得
    std::vector<bool> isHeight; // 高さ取得済みフラグ
    for (auto it = samplingLine.begin(); it != samplingLine.end(); it++)
    {
        std::set<double> height;
        CSearchOverlapRoads::ResultMap map = m_sor.Search(*it);
        if (roadPtr != nullptr)
        {
            // 道路の場合
            if (map.find(roadPtr) != map.end())
            {
                for (const auto &result : map[roadPtr])
                {
                    if (m_dataType == NETWORK_DATA_TYPE::ROADWAY
                        && result.nFunction != static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY)
                        && result.nFunction != static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::LANE)
                        && result.nFunction != static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION))
                        continue;

                    Boost3DPointHash crossPt;
                    if (CBoostGeoUtil::CalcVerticalCrossPt(*result.polygonPtr, *it, crossPt))
                    {
                        height.insert(crossPt.z());
                    }
                }
            }

            if (height.size() > 0)
            {
                if (m_dataType == NETWORK_DATA_TYPE::ROADWAY)
                    it->z(*height.begin());
                else
                    it->z(*height.rbegin());    // 歩道の場合で複数高さがある(段差)場合は、上の方の高さを取得する

                isHeight.push_back(true);
            }
            else
            {
                isHeight.push_back(false);
            }
        }
        else if (crossingPtr != nullptr)
        {
            // 横断歩道の場合
            for (const auto &results : map)
            {
                for (const auto &result : results.second)
                {
                    Boost3DPointHash crossPt;
                    if (CBoostGeoUtil::CalcVerticalCrossPt(*result.polygonPtr, *it, crossPt))
                    {
                        height.insert(crossPt.z());
                    }
                }
            }
            if (height.size() > 0)
            {
                it->z(*height.begin());
                isHeight.push_back(true);
            }
            else
            {
                isHeight.push_back(false);
            }
        }
    }

    // 勾配計測
    for (size_t i = 0; i < samplingLine.size() - 1; i++)
    {
        if (isHeight[i] && isHeight[i + 1])
        {
            CVector2D startPos2D = CBoostGeoUtil::ToCVector2D(samplingLine[i]);
            CVector2D endPos2D = CBoostGeoUtil::ToCVector2D(samplingLine[i + 1]);
            double dLen = (endPos2D - startPos2D).Length();
            dHorizontalDist += dLen;    // 平均縦断勾配用距離

            if (CEpsUtil::GreaterEqual(CUtil::RoundN(dLen, 3), dInterval)|| !bSamplingFlag)
            {
                double dVal;
                bool tmpIsEndHigher;
                if (CBoostGeoUtil::Gradient(samplingLine[i], samplingLine[i + 1], dVal, tmpIsEndHigher))
                {
                    n++;    // 計測数のカウント
                    CVector3D startPos = CBoostGeoUtil::ToCVector3D(samplingLine[i]);
                    CVector3D endPos = CBoostGeoUtil::ToCVector3D(samplingLine[i + 1]);
                    CVector3D vec = endPos - startPos;
                    CVector3D centerPos = vec * 0.5 + startPos;

                    if (CEpsUtil::Greater(dVal, dMax))
                    {
                        // 最大縦断勾配の更新
                        dMax = dVal;
                        isEndHigher = tmpIsEndHigher;
                        vSlopePos.x(centerPos.x);
                        vSlopePos.y(centerPos.y);
                        vSlopePos.z(centerPos.z);
                    }
                }
            }
        }
    }

    if (n > 0)
    {
        // 平均勾配
        double dAve = 0;
        size_t s = isHeight.size();
        size_t e = isHeight.size();
        for (int i = 0; i < isHeight.size(); i++)
        {
            if (isHeight[i] == true)
            {
                s = i;
                break;
            }
        }
        for (int i = isHeight.size() - 1; i >= 0; i--)
        {
            if (isHeight[i] == true)
            {
                e = i;
                break;
            }
        }

        if (s < isHeight.size() && e < isHeight.size())
        {
            dAve = abs(samplingLine[e].z() - samplingLine[s].z()) / dHorizontalDist;
            nAveVSlope = static_cast<int>(CUtil::RoundN(dAve * 100, 0));
        }
        dMaxVSlope = dMax;
        nMaxVSlope = static_cast<int>(CUtil::RoundN(dMax * 100, 0));
        bRet = true;
    }

    return bRet;
}

// 勾配の計測(横断歩道橋用)
bool CNetwork::calcGradient(
    const Boost3DHashPolyline &line,
    const std::shared_ptr<CBridgeData> &bridgePtr,
    const bool bSamplingFlag,
    const double dInterval,
    int &nMaxVSlope,
    double &dMaxVSlope,
    bool &isEndHigher,
    Boost3DPointHash &vSlopePos,
    int &nAveVSlope)
{
    bool bRet = false;
    double dMax = 0;            // 最大勾配
    int n = 0;                  // 計測数
    double dHorizontalDist = 0; // 平均勾配用

    if (bridgePtr != nullptr)
    {
        // 中心線を水平距離でサンプリング
        Boost3DHashPolyline samplingLine = bSamplingFlag ? CBoostGeoUtil::Sampling2D(line, dInterval) : line;

        // サンプリング点の高さを取得
        std::vector<bool> isHeight; // 高さ取得済みフラグ
        for (auto it = samplingLine.begin(); it != samplingLine.end(); it++)
        {
            std::set<double> height;
            CSearchOverlapBridge::ResultMap map = m_sob.Search(*it);
            if (map.find(bridgePtr) != map.end())
            {
                for (const auto &result : map[bridgePtr])
                {
                    Boost3DPointHash crossPt;
                    if (CBoostGeoUtil::CalcVerticalCrossPt(*result.polygonPtr, *it, crossPt))
                    {
                        height.insert(crossPt.z());
                    }
                }
            }

            if (height.size() > 0)
            {
                // OuterFloorSurfaceとOuterCeilingSurfaceから高さを取得しているため高い方(OuterFloorSurface)の高さを取得する
                it->z(*height.rbegin());
                isHeight.push_back(true);
            }
            else
            {
                isHeight.push_back(false);
            }
        }

        // 勾配計測
        for (size_t i = 0; i < samplingLine.size() - 1; i++)
        {
            if (isHeight[i] && isHeight[i + 1])
            {
                CVector2D startPos2D = CBoostGeoUtil::ToCVector2D(samplingLine[i]);
                CVector2D endPos2D = CBoostGeoUtil::ToCVector2D(samplingLine[i + 1]);
                double dLen = (endPos2D - startPos2D).Length();
                dHorizontalDist += dLen;    // 平均縦断勾配用距離
                if (CEpsUtil::GreaterEqual(CUtil::RoundN(dLen, 3), dInterval) || !bSamplingFlag)
                {
                    double dVal;
                    bool tmpIsEndHigher;
                    if (CBoostGeoUtil::Gradient(samplingLine[i], samplingLine[i + 1], dVal, tmpIsEndHigher))
                    {
                        n++;    // 計測数のカウント
                        CVector3D startPos = CBoostGeoUtil::ToCVector3D(samplingLine[i]);
                        CVector3D endPos = CBoostGeoUtil::ToCVector3D(samplingLine[i + 1]);
                        CVector3D vec = endPos - startPos;
                        CVector3D centerPos = vec * 0.5 + startPos;
                        if (CEpsUtil::Greater(dVal, dMax))
                        {
                            // 最大縦断勾配の更新
                            dMax = dVal;
                            isEndHigher = tmpIsEndHigher;
                            vSlopePos.x(centerPos.x);
                            vSlopePos.y(centerPos.y);
                            vSlopePos.z(centerPos.z);
                        }
                    }
                }
            }
        }

        if (n > 0)
        {
            // 平均勾配
            double dAve = 0;
            size_t s = isHeight.size();
            size_t e = isHeight.size();
            for (int i = 0; i < isHeight.size(); i++)
            {
                if (isHeight[i] == true)
                {
                    s = i;
                    break;
                }
            }
            for (int i = isHeight.size() - 1; i >= 0; i--)
            {
                if (isHeight[i] == true)
                {
                    e = i;
                    break;
                }
            }

            if (s < isHeight.size() && e < isHeight.size())
            {
                dAve = abs(samplingLine[e].z() - samplingLine[s].z()) / dHorizontalDist;
                nAveVSlope = static_cast<int>(CUtil::RoundN(dAve * 100, 0));
            }
            dMaxVSlope = dMax;
            nMaxVSlope = static_cast<int>(CUtil::RoundN(dMax * 100, 0));
            bRet = true;
        }
    }

    return bRet;
}

// 縦断勾配の計測(道路、横断歩道、横断歩道橋共通)
bool CNetwork::calcLongitudinalGradient(
    const Boost3DHashPolyline &line,
    const std::shared_ptr<CTranRoadData> &roadPtr,
    const std::shared_ptr<CFurnitureData> &crossingPtr,
    const std::shared_ptr<CBridgeData> &bridgePtr,
    const double dInterval,
    int &nMaxVSlope,
    bool &isEndHigher,
    Boost3DPointHash &vSlopePos,
    int &nAveVSlope)
{
    double dMaxVSlope;
    if (roadPtr != nullptr || crossingPtr != nullptr)
    {
        return calcGradient(
            line, roadPtr, crossingPtr, true, dInterval,
            nMaxVSlope, dMaxVSlope, isEndHigher, vSlopePos, nAveVSlope);
    }
    else
    {
        return calcGradient(
            line, bridgePtr, true, dInterval,
            nMaxVSlope, dMaxVSlope, isEndHigher, vSlopePos, nAveVSlope);
    }
}

 // 横断勾配の計測(道路、横断歩道、横断歩道橋共通)
bool CNetwork::calcCrossGradient(
    const Boost3DHashPolyline &line,
    const std::shared_ptr<CTranRoadData> &roadPtr,
    const std::shared_ptr<CFurnitureData> &crossingPtr,
    const std::shared_ptr<CBridgeData> &bridgePtr,
    const double dInterval,
    int &nMaxHSlope,
    Boost3DPointHash &hSlopePos)
{
    /** 横断勾配計算範囲は歩道幅(○●間)
     *  △     ：firstPoint
     *  ▲     ：secondPoint
     *  〇     ：rightCrossPoint
     *  ●     ：leftCrossPoint
     *  △->〇 ：rightVec
     *  △->● ：leftVec
     *  │     ：centerLine
     *  ┃     ：edge
     *
     * ●○間の勾配を計測する
     *
     * ┃ 　　　│　　　 ┃
     * ┃ 　　　▲　　　 ┃
     * ┃ 　　　│　　　 ┃
     * ┃ 　　　├┐　　 ┃
     * ●<- - - ┼┴ - ->〇
     * ┃ 　　　│　　　 ┃
     * ┃ 　　　│　　　 ┃
     * ┃ 　　　△　　　 ┃
     * ┃ 　　　　　　　 ┃
     * ┃<- roadWidth - >┃
     */

    bool bRet = false;
    Boost3DHashMultiLines targetLines;  // 横断勾配計測範囲(歩道幅の直線)
    std::vector<std::pair<CVector2D, CVector2D>> edgeVecPairList; // エッジの始点と方向ベクトルのペア

    // 横断勾配計測範囲の直線(歩道幅の直線)を取得
    if (roadPtr != nullptr)
    {
        // 歩道の場合は幅員計測の流用
        Boost3DHashPolygon targetPolygon;
        if (CTranRoadDataUtil::SearchFootpathPolygon(*roadPtr, line, targetPolygon))
        {
            // 抽出済みのエッジ線(中心線作成用)に注目歩道ポリゴンのエッジ線が存在するか確認する
            for (const auto &edgePair : roadPtr->m_footpath.edgePairList)
            {
                if (!CBoostGeoUtil::Disjoint(targetPolygon, edgePair.first)
                    && !CBoostGeoUtil::Disjoint(targetPolygon, edgePair.second))
                {
                    for (auto it = edgePair.first.cbegin(); it < edgePair.first.cend() - 1; it++)
                    {
                        CVector2D startPoint = CBoostGeoUtil::ToCVector2D(*it);
                        CVector2D endPoint = CBoostGeoUtil::ToCVector2D(*(it + 1));
                        CVector2D edgeVec = endPoint - startPoint;
                        edgeVecPairList.emplace_back(std::make_pair(edgeVec, startPoint));
                    }

                    for (auto it = edgePair.second.cbegin(); it < edgePair.second.cend() - 1; it++)
                    {
                        CVector2D startPoint = CBoostGeoUtil::ToCVector2D(*it);
                        CVector2D endPoint = CBoostGeoUtil::ToCVector2D(*(it + 1));
                        CVector2D edgeVec = endPoint - startPoint;
                        edgeVecPairList.emplace_back(std::make_pair(edgeVec, startPoint));
                    }
                }
            }

            if (edgeVecPairList.size() == 0)
            {
                // 歩道の輪郭線の始点と方向ベクトルを保存
                for (auto it = targetPolygon.outer().cbegin();
                    it < targetPolygon.outer().cend() - 1; it++)
                {
                    CVector2D startPoint = CBoostGeoUtil::ToCVector2D(*it);
                    CVector2D endPoint = CBoostGeoUtil::ToCVector2D(*(it + 1));
                    CVector2D edgeVec = endPoint - startPoint;
                    edgeVecPairList.emplace_back(std::make_pair(edgeVec, startPoint));
                }
            }
        }
    }
    else if (crossingPtr != nullptr)
    {
        // 横断歩道の場合はMBRを使用
        for (auto it = crossingPtr->m_pedestrianCrossingData.m_mbr.outer().cbegin();
            it < crossingPtr->m_pedestrianCrossingData.m_mbr.outer().cend() - 1; it++)
        {
            CVector2D startPoint = CBoostGeoUtil::ToCVector2D(*it);
            CVector2D endPoint = CBoostGeoUtil::ToCVector2D(*(it + 1));
            CVector2D edgeVec = endPoint - startPoint;
            edgeVecPairList.emplace_back(std::make_pair(edgeVec, startPoint));
        }
    }
    else if (bridgePtr != nullptr)
    {
        // 横断歩道橋の場合は、橋梁の輪郭線を使用
        for (auto it = bridgePtr->m_lod2List.front().m_boostGeometry.outer().cbegin();
            it < bridgePtr->m_lod2List.front().m_boostGeometry.outer().cend() - 1; it++)
        {
            CVector2D startPoint = CBoostGeoUtil::ToCVector2D(*it);
            CVector2D endPoint = CBoostGeoUtil::ToCVector2D(*(it + 1));
            CVector2D edgeVec = endPoint - startPoint;
            edgeVecPairList.emplace_back(std::make_pair(edgeVec, startPoint));
        }
    }

    if (edgeVecPairList.size() > 0)
    {
        auto sampledCenterLine = CBoostGeoUtil::Sampling(line, dInterval);

        // 中心線のセグメント毎に処理
        for (auto it = sampledCenterLine.cbegin();
            it < sampledCenterLine.cend() - 1; it++)
        {
            CVector2D firstPoint = CBoostGeoUtil::ToCVector2D(*it);
            CVector2D secondPoint = CBoostGeoUtil::ToCVector2D(*(it + 1));

            CVector2D inputVec = secondPoint - firstPoint;
            CVector2D rightVec;
            CVector2D leftVec;

            // 注目点(firstPoint)から左右に伸びる垂線ベクトルを求める
            if (CGeoUtil::GetVerticalVec(inputVec, rightVec) == false)
                continue;

            leftVec = -1 * rightVec;

            /// 左右の交点算出
            CVector2D rightCrossPoint;
            CVector2D leftCrossPoint;
            double rightCrossLength = LDBL_MAX;
            double leftCrossLength = LDBL_MAX;
            bool isRightCrossPoint = false;
            bool isLeftCrossPoint = false;

            // 歩道において歩道部ポリゴンから左右のエッジ候補作成した場合、
            // 隣接道路と重畳するエッジも残っているため、誤って交点を算出しないように
            // 注目中心線セグメントの中点から左右に垂線をおろす
            CVector2D centerPoint = inputVec * 0.5 + firstPoint;

            // 保存しなおしたセグメント毎に処理
            for (auto &edgeVecPair : edgeVecPairList)
            {
                bool isCross = false;
                CVector2D crossPointTmp;
                bool isOnLine1 = false;
                bool isOnLine2 = false;
                double t = 0.0;
                double s = 0.0;

                // 右側交点算出
                isCross = CGeoUtil::GetCrossPos(
                    rightVec, centerPoint,
                    edgeVecPair.first, edgeVecPair.second,
                    crossPointTmp, isOnLine1, isOnLine2, t, s);

                // 交点の取得成功
                // かつ、エッジセグメント上に交点がある
                // かつ、垂直方向に正の向きに交点がある
                // かつ、交点までの距離がより短い
                // かつ、交点までの距離が0ではない
                if (isCross == true
                    && isOnLine2 == true
                    && CEpsUtil::Greater(t, 0)
                    && CEpsUtil::Greater(rightCrossLength, t)
                    && !CEpsUtil::Zero(CUtil::RoundN(t, 3)))
                {

                    // 注目点(firstPoint)から一番近い交点を保存
                    rightCrossLength = t;
                    rightCrossPoint = crossPointTmp;
                    isRightCrossPoint = true;
                }

                // 左側交点算出
                isCross = CGeoUtil::GetCrossPos(
                    leftVec, centerPoint,
                    edgeVecPair.first, edgeVecPair.second,
                    crossPointTmp, isOnLine1, isOnLine2, t, s);

                // 交点の取得成功
                // かつ、エッジセグメント上に交点がある
                // かつ、垂直方向に正の向きに交点がある
                // かつ、交点までの距離がより短い
                // かつ、交点までの距離が0ではない
                if (isCross == true
                    && isOnLine2 == true
                    && CEpsUtil::Greater(t, 0)
                    && CEpsUtil::Greater(leftCrossLength, t)
                    && !CEpsUtil::Zero(CUtil::RoundN(t, 3)))
                {
                    // 注目点(firstPoint)から一番近い交点を保存
                    leftCrossLength = t;
                    leftCrossPoint = crossPointTmp;
                    isLeftCrossPoint = true;
                }
            }

            if (isRightCrossPoint && isLeftCrossPoint)
            {
                double dDist = rightCrossPoint.Distance(leftCrossPoint);
                if (CEpsUtil::GreaterEqual(dDist, dInterval))
                {
                    Boost3DHashPolyline line;
                    line.push_back(Boost3DPointHash(rightCrossPoint.x, rightCrossPoint.y, 0));
                    line.push_back(Boost3DPointHash(leftCrossPoint.x, leftCrossPoint.y, 0));
                    targetLines.push_back(line);
                }
            }
        }
    }

    // 最大横断勾配計測
    double dMaxHGradient = 0;
    for (const auto &targetLine : targetLines)
    {
        double dTmpSlope;
        int nTmpSlope, nAveSlope;
        Boost3DPointHash tmpPos;
        bool isEndHigher;
        bool bTmpRet;
        if (roadPtr != nullptr || crossingPtr != nullptr)
        {
            // 歩道, 横断歩道の場合
            bTmpRet = calcGradient(
                targetLine, roadPtr, crossingPtr, false, 0,
                nTmpSlope, dTmpSlope, isEndHigher, tmpPos, nAveSlope);
        }
        else
        {
            // 横断歩道橋の場合
            bTmpRet = calcGradient(
                targetLine, bridgePtr, false, 0,
                nTmpSlope, dTmpSlope, isEndHigher, tmpPos, nAveSlope);

        }

        if (bTmpRet)
        {
            if (CEpsUtil::Greater(dTmpSlope, dMaxHGradient))
            {
                dMaxHGradient = dTmpSlope;
                hSlopePos = tmpPos;
                bRet = true;
            }
        }
    }

    if (bRet)
    {
        nMaxHSlope = static_cast<int>(CUtil::RoundN(dMaxHGradient * 100, 0));
    }

    return bRet;
}

// リンク作成有無の確認
Boost3DMultiPointHashs CNetwork::checkLink(
    const std::shared_ptr<CTranRoadData> &tranPtr,
    const Boost3DHashMultiLines &centerLines)
{
    Boost3DMultiPointHashs pts;

    // 未作成リンクの確認
    if (m_dataType == NETWORK_DATA_TYPE::ROADWAY)
    {
        // 車道
        if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD1
            && !bg::is_empty(tranPtr->m_lod1.m_boostGeometry))
        {
            int nCnt = 0;
            for (const auto &line : centerLines)
            {
                if (!CBoostGeoUtil::Disjoint(tranPtr->m_lod1.m_boostGeometry, line))
                    nCnt++;
            }

            if (tranPtr->m_nInOut > 2)
            {
                // 交差点
                if (nCnt < tranPtr->m_nInOut)
                    pts.push_back(CErrLogger::GetInstance()->GetRefPt(tranPtr->m_lod1.m_boostGeometry));
            }
            else
            {
                if (nCnt == 0)
                    pts.push_back(CErrLogger::GetInstance()->GetRefPt(tranPtr->m_lod1.m_boostGeometry));
            }
        }
        else if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD2 || CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
        {
            int nCnt = 0;
            Boost3DMultiPointHashs tmpPts;
            BoostMultiPolygon tmpPolygon;
            if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD2)
            {
                for (const auto &lod2 : tranPtr->m_lod2List)
                {
                    tmpPolygon.push_back(CBoostGeoUtil::Conv(lod2.m_boostGeometry));
                    if (lod2.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY)
                        || lod2.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION))
                    {
                        bool bDisjoint = true;
                        for (const auto &line : centerLines)
                        {
                            if (!CBoostGeoUtil::Disjoint(lod2.m_boostGeometry, line))
                            {
                                nCnt++;
                                bDisjoint = false;
                            }
                        }
                        if (bDisjoint)
                            tmpPts.push_back(CErrLogger::GetInstance()->GetRefPt(lod2.m_boostGeometry));    // リンクなし地点
                    }
                }
            }
            else if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
            {
                for (const auto &lod3 : tranPtr->m_lod3List)
                {
                    tmpPolygon.push_back(CBoostGeoUtil::Conv(lod3.m_boostGeometry));
                    if ((m_dLod3Detail == 3.0 && lod3.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY))
                        || (m_dLod3Detail > 3.0 && lod3.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::LANE))
                        || lod3.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION))
                    {
                        bool bDisjoint = true;
                        for (const auto &line : centerLines)
                        {
                            if (!CBoostGeoUtil::Disjoint(lod3.m_boostGeometry, line))
                            {
                                nCnt++;
                                bDisjoint = false;
                            }
                        }
                        if (bDisjoint)
                            tmpPts.push_back(CErrLogger::GetInstance()->GetRefPt(lod3.m_boostGeometry));    // リンクなし地点
                    }
                }
            }

            if (tranPtr->m_nInOut > 2)
            {
                // 交差点
                if (nCnt < tranPtr->m_nInOut)
                {
                    BoostPoint pt;
                    bg::centroid(tmpPolygon, pt);
                    pts.push_back(Boost3DPointHash(pt.x(), pt.y(), 0));
                }
            }
            else
            {
                if (tmpPts.size() > 0)
                    pts.insert(pts.end(), tmpPts.begin(), tmpPts.end());
            }
        }
    }
    else if (m_dataType == NETWORK_DATA_TYPE::FOOTPATH)
    {
        // 歩道
        if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD2)
        {
            for (const auto &lod2 : tranPtr->m_lod2List)
            {
                if (lod2.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH))
                {
                    if (tranPtr->m_nInOut >= 3 && CEpsUtil::LessEqual(CBoostGeoUtil::Area(lod2.m_boostGeometry), 1.0))
                        continue;   // 交差点内の小面積歩道は無視

                    bool bDisjoint = true;
                    for (const auto &line : centerLines)
                    {
                        bDisjoint = CBoostGeoUtil::Disjoint(lod2.m_boostGeometry, line);
                        if (!bDisjoint)
                            break;
                    }
                    if (bDisjoint)
                    {
                        pts.push_back(CErrLogger::GetInstance()->GetRefPt(lod2.m_boostGeometry));
                    }
                }
            }
        }
        else if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
        {
            for (const auto &lod3 : tranPtr->m_lod3List)
            {
                if (lod3.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH))
                {
                    if (tranPtr->m_nInOut >= 3 && CEpsUtil::LessEqual(CBoostGeoUtil::Area(lod3.m_boostGeometry), 1.0))
                        continue;   // 交差点内の小面積歩道は無視

                    bool bDisjoint = true;
                    for (const auto &line : centerLines)
                    {
                        bDisjoint = CBoostGeoUtil::Disjoint(lod3.m_boostGeometry, line);
                        if (!bDisjoint)
                            break;
                    }
                    if (bDisjoint)
                    {
                        pts.push_back(CErrLogger::GetInstance()->GetRefPt(lod3.m_boostGeometry));
                    }
                }
            }
        }
    }
    return pts;
}
