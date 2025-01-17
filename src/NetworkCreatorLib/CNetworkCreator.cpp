#include "CNetworkCreator.h"

#include "CTranRoadDataUtil.h"
#include "CNetwork.h"
#include "CBridgeDataUtil.h"
#include "CFurnitureDataUtil.h"
#include "CGDALUtil.h"
#include "CBoostGeoUtil.h"
#include "CNearestNeighborSearch.h"
#include "CSearchOverlapRoads.h"
#include "COpenCVUtil.h"
#include "boost/format.hpp"
#include "CLogger.h"
#include "CErrLogger.h"
#include <thread>

#ifdef _DEBUG
#include "CDebugUtil.h"
#endif // _DEBUG

/*!
 * @brief CityObject配列を道路CityGMLに変換する
 * @para[in]  cityObjectList    CityGML配列
 * @para[in]  nJPZone           平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
 * @para[out] dLod3Detail       LOD3の場合の詳細度
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::SetTranRoadData(
    std::vector<const citygml::CityObject*>& cityObjectList, int nJPZone, double &dLod3Detail)
{
    dLod3Detail = 0;
    if (cityObjectList.size() < 1)
    {
        return false;
    }

    for (auto item : cityObjectList)
    {
        std::shared_ptr<CTranRoadData> tranRoadData = std::make_shared<CTranRoadData>();

        ///
        /// LOD1,2,3道路情報の取得
        ///
        bool bLod1Result, bLod2Result, bLod3Result;
        CTranRoadDataUtil::GetTranRoadData(
            item, *tranRoadData, nJPZone, bLod1Result, bLod2Result, bLod3Result);

        // tran:functionの取得
        CTranRoadDataUtil::GetTranFunction(item, *tranRoadData);
        // gml:nameの取得(路線名)
        CTranRoadDataUtil::GetGmlName(item, *tranRoadData);

        m_tranRoadData.emplace_back(tranRoadData);

        if (bLod3Result)
        {
            // LOD3の詳細度を確認
            // LOD3.0-LOD3.3の詳細度が混在することはないはずであるが、一応高詳細度を取得するようにする
            if (CEpsUtil::Greater(tranRoadData->m_lod3TriangularMeshList.front().m_dLod3Type, dLod3Detail))
                dLod3Detail = tranRoadData->m_lod3TriangularMeshList.front().m_dLod3Type;
        }
    }

    // LOD3の詳細度更新
    if (CEpsUtil::Greater(dLod3Detail, m_dLodType))
        m_dLodType = dLod3Detail;

    return true;
}

/*!
 * @brief CityObject配列を橋梁データに変換する
 * @param cityObjectList    CityGML配列
 * @param nJPZone           平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::SetBridgeData(std::vector<const citygml::CityObject *> &cityObjectList, int nJPZone)
{
    if (cityObjectList.size() < 1)
    {
        return false;
    }

    for (auto item : cityObjectList)
    {
        // 横断歩道橋データの取得
        std::shared_ptr<CBridgeData> bridgeData = std::make_shared<CBridgeData>();
        if (CBridgeDataUtil::GetPedestrianCrossingBridge(item, *bridgeData, nJPZone))
        {
            m_bridgeData.emplace_back(bridgeData);
        }
    }

    return true;
}

/*!
 * @brief CityObject配列を都市設備データに変換する
 * @param cityObjectList    CityGML配列
 * @param nJPZone           平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::SetFurnitureData(std::vector<const citygml::CityObject *> &cityObjectList, int nJPZone)
{
    if (cityObjectList.size() < 1)
    {
        return false;
    }

    for (auto item : cityObjectList)
    {
        // 横断歩道or点字ブロックデータの取得
        std::shared_ptr<CFurnitureData> furnitureData = std::make_shared<CFurnitureData>();
        double dLodType = 0;
        if (CFurnitureDataUtil::GetFurnitureDataQualityAttributeLodType(item, dLodType))
        {
            furnitureData->m_dLod3Type = dLodType;
        }
        if (CFurnitureDataUtil::GetPedestrianCrossingOrBrailleBlocks(item, *furnitureData, nJPZone))
        {
            if (furnitureData->m_functionType == FURNITURE_FUNCTION_TYPE::PEDESTRIAN_CROSSING)
            {
                // 横断歩道
                m_pedestrianCrossingData.emplace_back(furnitureData);
            }
            else if (furnitureData->m_functionType == FURNITURE_FUNCTION_TYPE::BRAILLE_BLOCKS)
            {
                // 点字ブロック
                m_brailleBlocksData.emplace_back(furnitureData);
            }
        }
    }
    return true;
}

/*!
 * @brief エッジ検出
 * @param lod    LOD
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::DetectEdge()
{
    for (auto tranRoadData : m_tranRoadData)
    {
        // 近傍道路探索
        if (tranRoadData->m_nInOut > 2)
        {
            // 近傍道路が2つより多い場合は交差点のため除外
            // LOD2以降は交差部でも島分割による車道ポリゴンも存在するが
            // 交差点接続でまとめて処理するためスキップ
            continue;
        }
        else if (tranRoadData->m_nInOut == 1)
        {
            // 近傍道路が1つのみの場合は端の道路として別処理をする
            std::vector<std::pair<Boost3DHashPolyline, Boost3DHashPolyline>> edgePairList;
            if (DetectEndOfRoadEdge(*tranRoadData, edgePairList))
            {
                for (auto edgePair : edgePairList)
                {
                    tranRoadData->edgePairList.emplace_back(edgePair);
                }
            }
            continue;
        }

        Boost3DHashMultiPolygon targetPolygonList;
        Boost3DHashMultiPolygon neighborPolygonList;
        bool isContainIsland = false;
        bool isUsingLanePolygon = false;

        // 使用するLODのデータを準備
        switch (m_iLod)
        {
        case 1:
            targetPolygonList.emplace_back(tranRoadData->m_lod1.m_boostGeometry);

            for (std::shared_ptr<CTranRoadData> neighborTranRoadDataPtrList : tranRoadData->m_neighborRoadPtr)
            {
                neighborPolygonList.emplace_back(neighborTranRoadDataPtrList->m_lod1.m_boostGeometry);
            }
            break;
        case 2:
            for (auto lod2 : tranRoadData->m_lod2List)
            {
                if (lod2.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY)
                {
                    targetPolygonList.emplace_back(lod2.m_boostGeometry);
                }
                else if (lod2.m_fuctionType == (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND)
                {
                    isContainIsland = true;
                }
            }

            for (std::shared_ptr<CTranRoadData> neighborTranRoadDataPtrList : tranRoadData->m_neighborRoadPtr)
            {
                for (auto lod2 : neighborTranRoadDataPtrList->m_lod2List)
                {
                    // 隣接道路は属性に関係なく取得する
                    neighborPolygonList.emplace_back(lod2.m_boostGeometry);
                }
            }
            break;
        case 3:
            for (auto lod3 : tranRoadData->m_lod3List)
            {
                if (CEpsUtil::Equal(lod3.m_dLod3Type, 3.0))
                {
                    if (lod3.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY)
                    {
                        targetPolygonList.emplace_back(lod3.m_boostGeometry);
                    }
                    else if (lod3.m_fuctionType == (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND)
                    {
                        isContainIsland = true;
                    }
                }
                else
                {
                    if (lod3.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::LANE)
                    {
                        targetPolygonList.emplace_back(lod3.m_boostGeometry);
                    }
                    else if (lod3.m_fuctionType == (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND)
                    {
                        isContainIsland = true;
                    }

                    isUsingLanePolygon = true;
                }
            }

            for (std::shared_ptr<CTranRoadData> neighborTranRoadDataPtrList : tranRoadData->m_neighborRoadPtr)
            {
                for (auto lod3 : neighborTranRoadDataPtrList->m_lod3List)
                {
                    // 隣接道路は属性に関係なく取得する
                    neighborPolygonList.emplace_back(lod3.m_boostGeometry);
                }
            }
            break;
        default:
            break;
        }

        if (targetPolygonList.size() < 1)
        {
            continue;
        }

        if (neighborPolygonList.size() < 1)
        {
            continue;
        }

        // 車道（交差点以外）の中に島がある場合は別処理をかける
        // ただし、LOD3.1以上の場合は車線が島の影響を受けないのでこの処理に入らない
        if (tranRoadData->m_nInOut < 3
            && isContainIsland
            && isUsingLanePolygon == false)
        {
            std::vector<std::pair<Boost3DHashPolyline, Boost3DHashPolyline>> edgePairList;
            if (DetectRoadWithIslandEdge(*tranRoadData, edgePairList))
            {
                for (auto edgePair : edgePairList)
                {
                    tranRoadData->edgePairList.emplace_back(edgePair);
                }
            }
            continue;
        }

        // ポリゴン毎にエッジ線を検出
        for (Boost3DHashPolygon targetPolygon : targetPolygonList)
        {
            std::pair<Boost3DHashPolyline, Boost3DHashPolyline> edgePair;
            if (ExtractRoadEdgeFromPolygon(targetPolygon, neighborPolygonList, edgePair))
            {
                tranRoadData->edgePairList.emplace_back(edgePair);
            }
        }
    }

    return true;
}

/*!
 * @brief ポリゴンからエッジを抽出
 * @param targetPolygonList     エッジ抽出する道路
 * @param neighborPolygonList   隣接道路
 * @param pairEdge              抽出したエッジのペア
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::ExtractRoadEdgeFromPolygon(
    Boost3DHashPolygon& targetPolygon,
    Boost3DHashMultiPolygon& neighborPolygonList,
    std::pair<Boost3DHashPolyline, Boost3DHashPolyline>& pairEdge)
{
    if (bg::is_empty(targetPolygon))
    {
        return false;
    }

    // 辺の重複リスト
    std::vector<bool> isEdgeList;
    isEdgeList.resize(targetPolygon.outer().size() - 1, true);

    // 近傍ポリゴンを総検索
    for (auto neighborPolygon : neighborPolygonList)
    {
        if (bg::is_empty(neighborPolygon))
        {
            continue;
        }

        std::vector<std::pair<size_t, size_t>> targetOverlapEdgeIndexPairList;
        if (IsOverlapPolygons(targetPolygon, neighborPolygon, targetOverlapEdgeIndexPairList))
        {
            for (auto overlapEdgeIdxPair : targetOverlapEdgeIndexPairList)
            {
                for (size_t idx = overlapEdgeIdxPair.first; idx < overlapEdgeIdxPair.second; idx++)
                {
                    isEdgeList[idx] = false;
                }
            }
        }
    }

    // 辺の重複リストからエッジを作成
    Boost3DHashPolyline edge;
    Boost3DHashMultiLines edges;
    int startIdx = 0;

    // 重複辺の位置をエッジ線作成の開始場所とする
    for (int i = 1; i < isEdgeList.size(); i++)
    {
        if (isEdgeList[i] == true && isEdgeList[i - 1] == false)
        {
            startIdx = i;
            break;
        }
    }

    // データの初めと最後が連続したエッジであることを考慮して作成する
    for (int i = 0; i < isEdgeList.size(); i++)
    {
        int targetIdx = (i + startIdx) % isEdgeList.size();
        int prevTargetIdx = (i - 1 + startIdx) % isEdgeList.size();

        if (isEdgeList[targetIdx] == true)
        {
            edge.emplace_back(targetPolygon.outer()[targetIdx]);
        }
        else if (isEdgeList[prevTargetIdx] == true)
        {
            edge.emplace_back(targetPolygon.outer()[targetIdx]);
            auto edgeLength = bg::length(edge);

            // 1m未満のエッジは対象外とする
            if (CEpsUtil::GreaterEqual(edgeLength, 1.0))
            {
                edges.emplace_back(edge);
            }

            edge.clear();
        }
    }

    // 2辺が取れた場合は道路のエッジとしてTranRoadDataに保存する
    if (edges.size() == 2)
    {
        pairEdge = std::make_pair(edges[0], edges[1]);
    }
    else
    {
        //std::pair<Boost3DHashPolyline, Boost3DHashPolyline> tmpPairEdge;

        //int resSplit = SplitEdges(targetPolygon, neighborPolygonList, tmpPairEdge);
        //if (resSplit == 2)
        //{
        //    pairEdge = tmpPairEdge;
        //}
        //else
        //{
        //    return false;
        //}

        return false;
    }

    return true;
}

/*!
 * @brief 誤検出したエッジの分割
 * @param tranRoadData      入力道路情報
 * @param targetPolygon     対象ポリゴン
 * @param isEdgeList        事前出力の重複リスト
 * @param edgePair          出力エッジペア
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
int CNetworkCreator::SplitEdges(
    Boost3DHashPolygon& targetPolygon,
    Boost3DHashMultiPolygon& neighborPolygonList,
    std::pair<Boost3DHashPolyline, Boost3DHashPolyline>& edgePair)
{
    if (bg::is_empty(targetPolygon))
    {
        return -1;
    }

    // 辺の重複リスト
    std::vector<bool> isEdgeList;
    isEdgeList.resize(targetPolygon.outer().size() - 1, true);

    // 近傍ポリゴンを総検索
    for (auto neighborPolygon : neighborPolygonList)
    {
        bool isEqualLineContinuously = true;

        for (int i = 0; i < targetPolygon.outer().size() - 1; i++)
        {
            // 重複しているか調べるポリライン
            Boost3DPointHash targetPoint = targetPolygon.outer()[i];

            bool isEqualLine = false;

            for (int ii = i + 1; ii < targetPolygon.outer().size(); ii++)
            {
                if (isEqualLineContinuously && ii >= targetPolygon.outer().size() - 1)
                {
                    // 初めの頂点から連続して辺が重複していて
                    // 最後の頂点まで達した場合は、
                    // 重複としない
                    // ex) 10の頂点からなるポリラインについて
                    // 0-1 と 1-9(始点0と同じ点)がどちらもtrueになってしまう
                    break;
                }

                Boost3DPointHash targetNextPoint = targetPolygon.outer()[ii];

                // 近傍ポリゴンの辺を総検索
                for (int j = 0; j < neighborPolygon.outer().size() - 1; j++)
                {
                    Boost3DPointHash searchPoint = neighborPolygon.outer()[j];

                    for (int jj = 0; jj < neighborPolygon.outer().size(); jj++)
                    {
                        Boost3DPointHash searchNextPoint = neighborPolygon.outer()[jj];

                        if ((targetPoint.IsRoundEqual(searchPoint) && targetNextPoint.IsRoundEqual(searchNextPoint))          // 同一方向
                            || (targetPoint.IsRoundEqual(searchNextPoint) && targetNextPoint.IsRoundEqual(searchPoint)))      // 逆方向

                        {
                            //// この一辺が完全重畳している場合
                            //isEdgeList[i] = false;
                            isEqualLine = true;
                            break;
                        }

                    }

                    if (isEqualLine)
                    {
                        break;
                    }
                }

                // 一致した辺に対して重複リストを更新
                if (isEqualLine)
                {
                    for (int idx = i; i < ii; i++)
                    {
                        isEdgeList[idx] = false;
                    }

                    // 次に調べるポリラインの頂点インデックスを調整
                    i = ii - 1;

                    break;
                }
            }

            if (isEqualLine == false)
            {
                isEqualLineContinuously = false;
            }
        }
    }

    // 辺の重複リストからエッジを作成
    Boost3DHashPolyline edge;
    Boost3DHashMultiLines edges;
    int startIdx = 0;

    // 重複辺の位置をエッジ線作成の開始場所とする
    for (int i = 1; i < isEdgeList.size(); i++)
    {
        if (isEdgeList[i] == true && isEdgeList[i - 1] == false)
        {
            startIdx = i;
            break;
        }
    }

    // データの初めと最後が連続したエッジであることを考慮して作成する
    for (int i = 0; i < isEdgeList.size(); i++)
    {
        int targetIdx = (i + startIdx) % isEdgeList.size();
        int prevTargetIdx = (i - 1 + startIdx) % isEdgeList.size();

        if (isEdgeList[targetIdx] == true)
        {
            edge.emplace_back(targetPolygon.outer()[targetIdx]);
        }
        else if (isEdgeList[prevTargetIdx] == true)
        {
            edge.emplace_back(targetPolygon.outer()[targetIdx]);
            edges.emplace_back(edge);
            edge.clear();
        }
    }

    // 2辺が取れたか確認
    if (edges.size() == 2)
    {
        edgePair = std::make_pair(edges[0], edges[1]);
    }

    return edges.size();
}

/*!
 * @brief 行き止まり道路のエッジ検出
 * @param tranRoadData      入力道路情報
 * @param edgePair          出力エッジペア
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::DetectEndOfRoadEdge(CTranRoadData& tranRoadData, std::vector<std::pair<Boost3DHashPolyline, Boost3DHashPolyline>>& edgePairList)
{
    if (tranRoadData.m_neighborRoadPtr.size() != 1)
    {
        return false;
    }

    Boost3DHashMultiPolygon targetPolygonList;
    Boost3DHashMultiPolygon neighborPolygonList;

    // 使用するLODのデータを準備
    switch (m_iLod)
    {
    case 1:
        targetPolygonList.emplace_back(tranRoadData.m_lod1.m_boostGeometry);

        for (std::shared_ptr<CTranRoadData> neighborTranRoadDataPtrList : tranRoadData.m_neighborRoadPtr)
        {
            neighborPolygonList.emplace_back(neighborTranRoadDataPtrList->m_lod1.m_boostGeometry);
        }
        break;
    case 2:
        for (auto lod2 : tranRoadData.m_lod2List)
        {
            if (lod2.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY)
            {
                targetPolygonList.emplace_back(lod2.m_boostGeometry);
            }
        }

        for (std::shared_ptr<CTranRoadData> neighborTranRoadDataPtrList : tranRoadData.m_neighborRoadPtr)
        {
            for (auto lod2 : neighborTranRoadDataPtrList->m_lod2List)
            {
                // 隣接道路は種別に関係なく追加する
                neighborPolygonList.emplace_back(lod2.m_boostGeometry);
            }
        }
        break;
    case 3:
        for (auto lod3 : tranRoadData.m_lod3List)
        {
            if (CEpsUtil::Equal(lod3.m_dLod3Type, 3.0))
            {
                if (lod3.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY)
                {
                    targetPolygonList.emplace_back(lod3.m_boostGeometry);
                }
            }
            else
            {
                if (lod3.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::LANE)
                {
                    targetPolygonList.emplace_back(lod3.m_boostGeometry);
                }
            }
        }

        for (std::shared_ptr<CTranRoadData> neighborTranRoadDataPtrList : tranRoadData.m_neighborRoadPtr)
        {
            for (auto lod3 : neighborTranRoadDataPtrList->m_lod3List)
            {
                // 隣接道路は種別に関係なく追加する
                neighborPolygonList.emplace_back(lod3.m_boostGeometry);
            }
        }
        break;
    default:
        break;
    }

    // ポリゴン毎にエッジ線を検出
    for (Boost3DHashPolygon targetPolygon : targetPolygonList)
    {
        if (bg::is_empty(targetPolygon))
        {
            continue;
        }

        // エッジの内、隣接ポリゴンと接しているか調べる
        std::vector<bool> isOverlapEdgeList;
        isOverlapEdgeList.resize(targetPolygon.outer().size() - 1, false);

        // 他ポリゴンと重複していないか確認
        std::vector<std::pair<size_t, size_t>> targetOverlapEdgeIndexPairList;
        if (IsOverlapPolygons(targetPolygon, neighborPolygonList, targetOverlapEdgeIndexPairList))
        {
            for (auto overlapEdgeIdxPair : targetOverlapEdgeIndexPairList)
            {
                for (size_t idx = overlapEdgeIdxPair.first; idx < overlapEdgeIdxPair.second; idx++)
                {
                    if (idx < targetPolygon.outer().size() - 1)
                    {
                        isOverlapEdgeList[idx] = true;
                    }
                }
            }
        }

        // サンプリング後のにエッジ情報を振り分ける
        //  1 : エッジ1
        //  2 : エッジ対象外1
        //  0 : エッジ未発見
        // -1 : エッジ2（エッジ1と反対側のエッジ）
        // -2 : エッジ対象外2
        // -99: 隣接辺
        std::vector<std::pair<int, CVector2D>> sampledEdgeInfoList;

        // 行き止まり道路のポリゴンをサンプリングする
        Boost3DHashPolygon sampledTargetPolygon;
        for (int i = 0; i < targetPolygon.outer().size() - 1; i++)
        {
            Boost3DPointHash startPoint = targetPolygon.outer()[i];
            Boost3DPointHash endPoint = targetPolygon.outer()[i + 1];

            CVector2D edgeVec = CVector2D(endPoint.x(), endPoint.y()) - CVector2D(startPoint.x(), startPoint.y());

            // 隣接ポリゴンと重畳している辺はサンプリングせずにスキップ
            if (isOverlapEdgeList[i] == true)
            {
                // サンプリング後のポリゴンに保存
                // ポリライン始点は前セグメントの終点と一致するため
                // ポリライン終点のみ保存する

                // 最初の点であればポリゴンの頂点を追加
                // 一つ前の点と異なる座標であればポリゴンの頂点を追加
                if (sampledTargetPolygon.outer().size() == 0)
                {
                    sampledTargetPolygon.outer().emplace_back(startPoint);

                    // サンプリング後のエッジ情報に保存
                    sampledEdgeInfoList.emplace_back(std::make_pair(-99, edgeVec));
                }
                else if (sampledTargetPolygon.outer().back().IsRoundEqual(startPoint) == false)
                {
                    sampledTargetPolygon.outer().emplace_back(startPoint);

                    // サンプリング後のエッジ情報に保存
                    sampledEdgeInfoList.erase(sampledEdgeInfoList.cend() - 1);
                    sampledEdgeInfoList.emplace_back(std::make_pair(-99, edgeVec));
                    sampledEdgeInfoList.emplace_back(std::make_pair(-99, edgeVec));
                }

                // 一つ前の点と異なる座標であればポリゴンの頂点を追加
                if (sampledTargetPolygon.outer().back().IsRoundEqual(endPoint) == false)
                {
                    sampledTargetPolygon.outer().emplace_back(endPoint);

                    // サンプリング後のエッジ情報に保存
                    sampledEdgeInfoList.erase(sampledEdgeInfoList.cend() - 1);
                    sampledEdgeInfoList.emplace_back(std::make_pair(-99, edgeVec));
                    sampledEdgeInfoList.emplace_back(std::make_pair(-99, edgeVec));
                }

                continue;
            }

            // ポリラインを取得
            Boost3DHashPolyline polyline;
            polyline.emplace_back(startPoint);
            polyline.emplace_back(endPoint);

            // サンプリング
            // 詳細にサンプリングせずに数点頂点を追加する
            Boost3DHashPolyline sampledPolyline = CBoostGeoUtil::Sampling(polyline, (int)2);

            for (auto item : sampledPolyline)
            {
                // 最初の点であればポリゴンの頂点を追加
                // 一つ前の点と異なる座標であればポリゴンの頂点を追加
                if (sampledTargetPolygon.outer().size() == 0)
                {
                    sampledTargetPolygon.outer().emplace_back(item);

                    // 同時にサンプリング後のエッジ情報を保存
                    sampledEdgeInfoList.emplace_back(std::make_pair(0, edgeVec));
                }
                else if (sampledTargetPolygon.outer().back().IsRoundEqual(item) == false)
                {
                    sampledTargetPolygon.outer().emplace_back(item);

                    // 同時にサンプリング後のエッジ情報を保存
                    sampledEdgeInfoList.erase(sampledEdgeInfoList.cend() - 1);
                    sampledEdgeInfoList.emplace_back(std::make_pair(0, edgeVec));
                    sampledEdgeInfoList.emplace_back(std::make_pair(0, edgeVec));
                }
            }
        }

        int startIdx = 0;

        // 重複辺の位置をエッジ線作成の開始場所とする
        for (int i = 1; i < sampledEdgeInfoList.size(); i++)
        {
            if (sampledEdgeInfoList[i].first != -99
                && sampledEdgeInfoList[i - 1].first == -99)
            {
                startIdx = i;
                break;
            }
        }

        // 始点と終点からセグメントの平行確認
        for (int i = 0; i < sampledEdgeInfoList.size(); i++)
        {
            int targetIdx = (i + startIdx) % sampledEdgeInfoList.size();

            // すでに平行な線が見つかっているか
            // 他ポリゴンと重複している場合は
            // スキップ
            if (sampledEdgeInfoList[targetIdx].first != 0)
            {
                continue;
            }

            CVector2D startVec = sampledEdgeInfoList[targetIdx].second;

            for (int j = sampledEdgeInfoList.size() - 1; j >= 0; j--)
            {
                int searchIdx = (j + startIdx) % sampledEdgeInfoList.size();

                // すでに平行な線が見つかっているか
                // 他ポリゴンと重複している場合は
                // スキップ
                if (sampledEdgeInfoList[searchIdx].first != 0)
                {
                    continue;
                }

                CVector2D endVec = sampledEdgeInfoList[searchIdx].second;

                if (IsParallel(startVec, endVec, -1))
                {
                    sampledEdgeInfoList[targetIdx].first = 1;
                    sampledEdgeInfoList[searchIdx].first = -1;

                    // 平行な辺の巻き戻りが発生しないように
                    // 以前の辺の内、平行未発見のものは検索対象外にする
                    for (int ii = 0; ii < i; ii++)
                    {
                        int tmpTargetIdx = (ii + startIdx) % sampledEdgeInfoList.size();

                        // すでに平行な線が見つかっているか
                        // 他ポリゴンと重複している場合は
                        // スキップ
                        if (sampledEdgeInfoList[tmpTargetIdx].first == 0)
                        {
                            sampledEdgeInfoList[tmpTargetIdx].first = 2;
                        }
                    }
                    for (int jj = sampledEdgeInfoList.size() - 1; jj >= j; jj--)
                    {
                        int tmpSearchIdx = (jj + startIdx) % sampledEdgeInfoList.size();

                        // すでに平行な線が見つかっているか
                        // 他ポリゴンと重複している場合は
                        // スキップ
                        if (sampledEdgeInfoList[tmpSearchIdx].first == 0)
                        {
                            sampledEdgeInfoList[tmpSearchIdx].first = -2;
                        }
                    }

                    break;
                }
            }
        }

        Boost3DHashPolyline edge1;
        Boost3DHashPolyline edge2;

        int beforeEdgeInfo = 0;
        bool isNotEdge = false;
        for (int i = 0; i < sampledEdgeInfoList.size(); i++)
        {
            int targetIdx = (i + startIdx) % sampledEdgeInfoList.size();
            int currentEdgeInfo = sampledEdgeInfoList[targetIdx].first;

            if (currentEdgeInfo == 1
                || currentEdgeInfo == 2)
            {
                edge1.emplace_back(sampledTargetPolygon.outer()[targetIdx]);

                if (isNotEdge == false
                    && (beforeEdgeInfo == -1 || beforeEdgeInfo == -2))
                {
                    edge2.emplace_back(sampledTargetPolygon.outer()[targetIdx]);
                }

                isNotEdge = false;
            }
            else if (currentEdgeInfo == -1
                || currentEdgeInfo == -2)
            {
                edge2.emplace_back(sampledTargetPolygon.outer()[targetIdx]);

                if (isNotEdge == false
                    && (beforeEdgeInfo == 1 || beforeEdgeInfo == 2))
                {
                    edge1.emplace_back(sampledTargetPolygon.outer()[targetIdx]);
                }

                isNotEdge = false;
            }
            else if (currentEdgeInfo == 0)
            {
                if (isNotEdge == false)
                {
                    // 一旦現在のポイントをエッジに保存する
                    if (beforeEdgeInfo == 1
                        || beforeEdgeInfo == 2)
                    {
                        edge1.emplace_back(sampledTargetPolygon.outer()[targetIdx]);
                    }
                    else if (beforeEdgeInfo == -1
                        || beforeEdgeInfo == -2)
                    {
                        edge2.emplace_back(sampledTargetPolygon.outer()[targetIdx]);
                    }

                    isNotEdge = true;
                }

                // beforeEdgeInfoも更新せずに
                // スキップ
                continue;
            }
            else
            {
                if (isNotEdge == false)
                {
                    // 一旦現在のポイントをエッジに保存する
                    if (beforeEdgeInfo == 1
                        || beforeEdgeInfo == 2)
                    {
                        edge1.emplace_back(sampledTargetPolygon.outer()[targetIdx]);
                    }
                    else if (beforeEdgeInfo == -1
                        || beforeEdgeInfo == -2)
                    {
                        edge2.emplace_back(sampledTargetPolygon.outer()[targetIdx]);
                    }

                    isNotEdge = true;
                }

                // スキップ
                // beforeEdgeInfoは更新する
            }

            beforeEdgeInfo = currentEdgeInfo;
        }

        if (bg::is_empty(edge1) == false
            && bg::is_empty(edge2) == false)
        {
            edgePairList.emplace_back(std::make_pair(edge1, edge2));
        }

    }

    if (edgePairList.size() > 0)
    {
        return true;
    }

    return false;
}

/*!
 * @brief LOD2以上で島がある道路のエッジ検出
 * @param tranRoadData      入力道路情報
 * @param edgePair          出力エッジペア
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::DetectRoadWithIslandEdge(CTranRoadData& tranRoadData, std::vector<std::pair<Boost3DHashPolyline, Boost3DHashPolyline>>& edgePairList)
{
    edgePairList.clear();

    Boost3DHashMultiPolygon targetPolygonList;
    Boost3DHashMultiPolygon footpathPolygonList;
    Boost3DHashMultiPolygon islandPolygonList;
    Boost3DHashMultiPolygon neighborPolygonList;

    // 使用するLODのデータを準備
    switch (m_iLod)
    {
    case 1:
        // LOD1は島が存在しないため失敗
        return false;
    case 2:
        for (auto lod2 : tranRoadData.m_lod2List)
        {
            if (lod2.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY)
            {
                targetPolygonList.emplace_back(lod2.m_boostGeometry);
            }
            else if (lod2.m_fuctionType == (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND)
            {
                islandPolygonList.emplace_back(lod2.m_boostGeometry);
            }
            else if (lod2.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH)
            {
                // 島と隣接している歩道は島と同等の処理をする
                bool isNeighborIsland = false;
                for (auto tmpLod2 : tranRoadData.m_lod2List)
                {
                    double dist = bg::distance(CBoostGeoUtil::Conv(lod2.m_boostGeometry), CBoostGeoUtil::Conv(tmpLod2.m_boostGeometry));

                    if (CEpsUtil::Equal(dist, 0.0)
                        && tmpLod2.m_fuctionType == (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND)
                    {
                        isNeighborIsland = true;
                        break;
                    }
                }
                if (isNeighborIsland)
                {
                    islandPolygonList.emplace_back(lod2.m_boostGeometry);
                }
                else
                {
                    footpathPolygonList.emplace_back(lod2.m_boostGeometry);
                }
            }
        }

        for (std::shared_ptr<CTranRoadData> neighborTranRoadDataPtrList : tranRoadData.m_neighborRoadPtr)
        {
            for (auto lod2 : neighborTranRoadDataPtrList->m_lod2List)
            {
                neighborPolygonList.emplace_back(lod2.m_boostGeometry);
            }
        }
        break;
    case 3:
        for (auto lod3 : tranRoadData.m_lod3List)
        {
            if (CEpsUtil::Equal(lod3.m_dLod3Type, 3.0))
            {
                if (lod3.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY)
                {
                    targetPolygonList.emplace_back(lod3.m_boostGeometry);
                }
                else if (lod3.m_fuctionType == (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND)
                {
                    islandPolygonList.emplace_back(lod3.m_boostGeometry);
                }
                else if (lod3.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH)
                {
                    // 島と隣接している歩道は島と同等の処理をする
                    bool isNeighborIsland = false;
                    for (auto tmpLod3 : tranRoadData.m_lod3List)
                    {
                        double dist = bg::distance(CBoostGeoUtil::Conv(lod3.m_boostGeometry), CBoostGeoUtil::Conv(tmpLod3.m_boostGeometry));

                        if (CEpsUtil::Equal(dist, 0.0)
                            && tmpLod3.m_fuctionType == (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND)
                        {
                            isNeighborIsland = true;
                            break;
                        }
                    }
                    if (isNeighborIsland)
                    {
                        islandPolygonList.emplace_back(lod3.m_boostGeometry);
                    }
                    else
                    {
                        footpathPolygonList.emplace_back(lod3.m_boostGeometry);
                    }
                }
            }
            else
            {
                if (lod3.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::LANE)
                {
                    targetPolygonList.emplace_back(lod3.m_boostGeometry);
                }
                else if (lod3.m_fuctionType == (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND)
                {
                    islandPolygonList.emplace_back(lod3.m_boostGeometry);
                }
                else if (lod3.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH)
                {
                    // 島と隣接している歩道は島と同等の処理をする
                    bool isNeighborIsland = false;
                    for (auto tmpLod3 : tranRoadData.m_lod3List)
                    {
                        double dist = bg::distance(CBoostGeoUtil::Conv(lod3.m_boostGeometry), CBoostGeoUtil::Conv(tmpLod3.m_boostGeometry));

                        if (CEpsUtil::Equal(dist, 0.0)
                            && tmpLod3.m_fuctionType == (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND)
                        {
                            isNeighborIsland = true;
                            break;
                        }
                    }
                    if (isNeighborIsland)
                    {
                        islandPolygonList.emplace_back(lod3.m_boostGeometry);
                    }
                    else
                    {
                        footpathPolygonList.emplace_back(lod3.m_boostGeometry);
                    }
                }
            }
        }

        for (std::shared_ptr<CTranRoadData> neighborTranRoadDataPtrList : tranRoadData.m_neighborRoadPtr)
        {
            for (auto lod3 : neighborTranRoadDataPtrList->m_lod3List)
            {
                neighborPolygonList.emplace_back(lod3.m_boostGeometry);
            }
        }
        break;
    default:
        break;
    }

    // 車道ポリゴンが存在しない場合は失敗
    if (targetPolygonList.size() == 0
        || neighborPolygonList.size() == 0)
    {
        return false;
    }

    // 島がないのにポリゴンが複数ある場合（その場合はそもそもここに入らないし存在しない気がするが）
    // 各ポリゴンごとにエッジ処理をすればよいのでreturn false
    if (islandPolygonList.size() < 1)
    {
        return false;
    }

    // 島があってそれによりポリゴンが二分割されている場合
    // 島との境界を求めて、それぞれのポリゴンのエッジとして使用する
    if (targetPolygonList.size() > 1)
    {
        // 島なしと同様、道路ポリゴンからエッジを検出する
        for (Boost3DHashPolygon targetPolygon : targetPolygonList)
        {
            std::pair<Boost3DHashPolyline, Boost3DHashPolyline> edgePair;
            if (ExtractRoadEdgeFromPolygon(targetPolygon, neighborPolygonList, edgePair))
            {
                edgePairList.emplace_back(edgePair);
            }
        }

        return true;
    }

    // 島があるが途中で切れていたり長さが短かったりして車道が分割されず一つになっている場合
    // 島のエッジを推測して作成し、分割されたポリゴンについてエッジを作成する
    if (targetPolygonList.size() == 1)
    {
        ///
        /// 全体のエッジを検出する
        ///

        // 車道と島を融合する
        Boost3DHashMultiPolygon roadwayAndIslandPolygons;
        Boost3DHashMultiPolygon dissolvedRoadwayAndIslandPolygons;
        for (auto polygon : targetPolygonList)
        {
            roadwayAndIslandPolygons.emplace_back(polygon);
        }
        for (auto polygon : islandPolygonList)
        {
            roadwayAndIslandPolygons.emplace_back(polygon);
        }
        roadwayAndIslandPolygons.size() > 1 ? dissolvedRoadwayAndIslandPolygons = CGDALUtil::GetInstance()->Dissolve(roadwayAndIslandPolygons) : dissolvedRoadwayAndIslandPolygons = roadwayAndIslandPolygons;

        std::vector<std::pair<Boost3DHashPolyline, Boost3DHashPolyline>> tmpEdgePairList;
        for (auto dissolvedPolygon : dissolvedRoadwayAndIslandPolygons)
        {
            std::pair<Boost3DHashPolyline, Boost3DHashPolyline> edgePair;
            if (ExtractRoadEdgeFromPolygon(dissolvedPolygon, neighborPolygonList, edgePair))
            {
                tmpEdgePairList.emplace_back(edgePair);
            }
        }

        if (tmpEdgePairList.size() == 0)
        {
            return false;
        }

        /////
        ///// 島を無視して中心線を作成する
        /////

        Boost3DHashPolyline centerLineWithoutIsland;
        if (CreatePolylineBetweenTwoPolylines(tmpEdgePairList[0], centerLineWithoutIsland) == false)
        {
            return false;
        }

        ///
        /// 島をグルーピングする
        ///

        // 島のグルーピング結果
        std::vector<Boost3DHashMultiPolygon> islandGroupingList;
        Boost3DHashMultiPolygon dissolvedIslandGroupingList;

        // グルーピングに必要な島ごとの情報
        std::vector<std::tuple<double, double, size_t>> islandInfoList; // tmpEdgePairListのインデックス、全体エッジまでの距離1、全体エッジまでの距離2、グルーピングインデックス

        // グルーピング数
        int groupNum = 0;

        // 島ごとに各エッジまでの距離を求める
        for(int i = 0; i < islandPolygonList.size(); i++)
        {
            auto polygon = islandPolygonList[i];
            std::tuple<double, double, size_t> islandInfo = std::make_tuple(DBL_MAX, DBL_MAX, 0);

            Boost3DPointHash centroid;
            bg::centroid(polygon, centroid);

            for (int j = 0; j < tmpEdgePairList.size(); j++)
            {
                auto tmpEdgePair = tmpEdgePairList[j];

                // 距離を算出
                auto dist1 = bg::distance(CBoostGeoUtil::Conv(centroid), CBoostGeoUtil::Conv(tmpEdgePair.first));
                auto dist2 = bg::distance(CBoostGeoUtil::Conv(centroid), CBoostGeoUtil::Conv(tmpEdgePair.second));

                if (CEpsUtil::Less(dist1 + dist2, std::get<0>(islandInfo) + std::get<1>(islandInfo)))
                {
                    islandInfo = std::make_tuple(dist1, dist2, 0);
                }
            }

            // 最初はグルーピングインデックスを設定して追加する
            if (islandInfoList.size() == 0)
            {
                islandInfo = std::make_tuple(std::get<0>(islandInfo), std::get<1>(islandInfo), 0);
                islandInfoList.emplace_back(islandInfo);
                groupNum++;

                continue;
            }

            // グルーピングインデックスは新規追加する用の番号を仮設定する
            islandInfo = std::make_tuple(std::get<0>(islandInfo), std::get<1>(islandInfo), groupNum);
            groupNum++;

            // すでに情報が入っている場合、すでにある情報から同じグループを判別する
            for (int j = 0; j < islandInfoList.size(); j++)
            {
                auto tmpIslandInfo = islandInfoList[j];

                double diff1 = std::abs(std::get<0>(islandInfo) - std::get<0>(tmpIslandInfo));
                double diff2 = std::abs(std::get<1>(islandInfo) - std::get<1>(tmpIslandInfo));

                double toEdgeDistanceThreshold = 1.0;

                if (CEpsUtil::LessEqual(diff1, toEdgeDistanceThreshold)
                    || CEpsUtil::LessEqual(diff2, toEdgeDistanceThreshold))
                {
                    // 全体エッジまでの距離が等しい場合
                    // グルーピングインデックスを設定する
                    islandInfo = std::make_tuple(std::get<0>(islandInfo), std::get<1>(islandInfo), std::get<2>(tmpIslandInfo));
                    groupNum--;

                    break;
                }
            }

            islandInfoList.emplace_back(islandInfo);
        }

        // 島ごとの情報を基にグルーピングする
        for (int i = 0; i < islandInfoList.size(); i++)
        {
            auto islandInfo = islandInfoList[i];

            if (islandGroupingList.size() > std::get<2>(islandInfo))
            {
                islandGroupingList[std::get<2>(islandInfo)].emplace_back(islandPolygonList[i]);
            }
            else
            {
                Boost3DHashMultiPolygon insertMultiPolygon;
                insertMultiPolygon.emplace_back(islandPolygonList[i]);
                islandGroupingList.emplace_back(insertMultiPolygon);
            }
        }

        ///
        /// グルーピング毎に島の長さを計測して
        /// 道路長に対してどの程度であるか測定
        ///
        for (int i = 0; i < islandGroupingList.size(); i++)
        {
            auto islandGroupingPolygons = islandGroupingList[i];

            double islandLengthRate = 0.0;
            double islandLengthRateThreshold = 0.7; // TODO パラメータ化
            double islandLength = 0.0;
            double centerLineLength = bg::length(centerLineWithoutIsland);

            if (centerLineLength == 0.0)
            {
                return false;
            }

            for (auto polygon : islandGroupingPolygons)
            {
                for(int i = 0; i < polygon.outer().size()- 1; i++)
                {
                    Boost3DHashPolyline line;
                    line.emplace_back(polygon.outer()[i]);
                    line.emplace_back(polygon.outer()[i + 1]);

                    islandLength += bg::length(line);
                }
            }

            islandLengthRate = (islandLength / 2) / centerLineLength;

            // 閾値未満の場合は
            // 対象の島グループを削除
            if (CEpsUtil::Less(islandLengthRate, islandLengthRateThreshold))
            {
                islandGroupingList.erase(islandGroupingList.cbegin() + i);
                i--;

            }
        }

        ///
        /// グルーピング毎にエッジを作成
        ///
        std::vector<std::pair<Boost3DHashPolyline, double>> createdEdgeList; // エッジ、道路の片方の端からの距離
        for (int i = 0; i < islandGroupingList.size(); i++)
        {
            // 島を無視した中心線のインデックス検索リストを作る
            // 中心線の真ん中から外側に向かって順にインデックスを取得し
            // 島と車道の境界を探す
            std::vector<size_t> searchCenterLineIndexList;

            size_t startIdx = (centerLineWithoutIsland.size() - 1) / 2;
            searchCenterLineIndexList.emplace_back(startIdx);

            int signNum = -1;
            for (int i = 1; i < centerLineWithoutIsland.size() - 1; i++)
            {
                searchCenterLineIndexList.emplace_back(searchCenterLineIndexList.back() + (signNum * i));
                signNum *= -1;
            }

            // 島を無視した中心線から
            // 島と車道の境界までの差分ベクトルを求める
            CVector2D centerLineWithoutIsland2IslandEdge1;
            CVector2D centerLineWithoutIsland2IslandEdge2;
            for (auto idx : searchCenterLineIndexList)
            {
                Boost3DPointHash firstPoint = centerLineWithoutIsland[idx];
                Boost3DPointHash secondPoint = centerLineWithoutIsland[idx + 1];

                // 島を無視した中心線の一つのセグメントを取得
                CVector2D firstPointVector2D = CVector2D(firstPoint.x(), firstPoint.y());
                CVector2D secondPointVector2D = CVector2D(secondPoint.x(), secondPoint.y());
                CVector2D segmentVec2D = secondPointVector2D - firstPointVector2D;
                CVector2D centerSegmentVec2D = CVector2D((firstPointVector2D.x + secondPointVector2D.x) / 2, (firstPointVector2D.y + secondPointVector2D.y) / 2);

                // セグメントに垂直なベクトルを算出
                CVector2D verticalVector2D;
                if (CGeoUtil::GetVerticalVec(segmentVec2D, verticalVector2D) == false)
                {
                    // 次のセグメントへ
                    continue;
                }
                verticalVector2D.Normalize();

                // 島を構成するすべてのセグメントを検索し
                // 交点を探す
                bool isCrossed1 = false;
                bool isCrossed2 = false;
                for (auto islandPolygon : islandGroupingList[i])
                {
                    for (int i = 0; i < islandPolygon.outer().size() - 1; i++)
                    {
                        Boost3DPointHash islandFirstPoint = islandPolygon.outer()[i];
                        Boost3DPointHash islandSecondPoint = islandPolygon.outer()[i + 1];

                        // 島を構成する一つのセグメントを取得
                        CVector2D islandFirstPointVector2D = CVector2D(islandFirstPoint.x(), islandFirstPoint.y());
                        CVector2D islandSecondPointVector2D = CVector2D(islandSecondPoint.x(), islandSecondPoint.y());
                        CVector2D islandSegmentVec2D = islandSecondPointVector2D - islandFirstPointVector2D;

                        CVector2D crossPointVector2D;
                        bool isOnLine1;
                        bool isOnLine2;
                        double t = 0.0;
                        double s = 0.0;
                        if (CGeoUtil::GetCrossPos(
                            verticalVector2D,
                            centerSegmentVec2D,
                            islandSegmentVec2D,
                            islandFirstPointVector2D,
                            crossPointVector2D,
                            isOnLine1,
                            isOnLine2,
                            t,
                            s)
                            && isOnLine2
                            && s >= 0.0
                            && s <= 1.0)
                        {
                            if (isCrossed1)
                            {
                                centerLineWithoutIsland2IslandEdge2 = verticalVector2D * t;

                                isCrossed2 = true;
                            }
                            else
                            {
                                centerLineWithoutIsland2IslandEdge1 = verticalVector2D * t;

                                isCrossed1 = true;
                            }
                        }

                        if (isCrossed1
                            && isCrossed2)
                        {
                            // 交点が2点見つかったら終了
                            break;
                        }
                    }

                    if (isCrossed1
                        && isCrossed2)
                    {
                        // 交点が2点見つかったら終了
                        break;
                    }
                }

                if (isCrossed1
                    && isCrossed2)
                {
                    // 交点が2点見つかったら終了
                    break;
                }
            }

            // 島のエッジ（切れ目補間済み）を決定する
            Boost3DHashPolyline islandEdge1;
            Boost3DHashPolyline islandEdge2;
            for (auto point : centerLineWithoutIsland)
            {
                Boost3DPointHash newPoint1 = Boost3DPointHash(point.x() + centerLineWithoutIsland2IslandEdge1.x, point.y() + centerLineWithoutIsland2IslandEdge1.y, point.z());
                islandEdge1.emplace_back(newPoint1);

                Boost3DPointHash newPoint2 = Boost3DPointHash(point.x() + centerLineWithoutIsland2IslandEdge2.x, point.y() + centerLineWithoutIsland2IslandEdge2.y, point.z());
                islandEdge2.emplace_back(newPoint2);
            }

            // エッジが取得出来たら
            // 端からの距離を算出して保存する
            if (islandEdge1.size() > 0
                || islandEdge2.size() > 0)
            {
                auto dist1 = bg::distance(CBoostGeoUtil::Conv(islandEdge1), CBoostGeoUtil::Conv(tmpEdgePairList[0].first));
                auto dist2 = bg::distance(CBoostGeoUtil::Conv(islandEdge2), CBoostGeoUtil::Conv(tmpEdgePairList[0].first));

                createdEdgeList.emplace_back(std::make_pair(islandEdge1, dist1));
                createdEdgeList.emplace_back(std::make_pair(islandEdge2, dist2));
            }
        }

        ///
        /// 取得したエッジを端から順に並び変えて
        /// ペアを作成する
        ///

        // 初めに作成した道路全体のエッジを追加する
        for (auto polylinePair : tmpEdgePairList)
        {
            auto firstDistance = bg::distance(CBoostGeoUtil::Conv(polylinePair.first), CBoostGeoUtil::Conv(tmpEdgePairList[0].first));
            auto secondDistance = bg::distance(CBoostGeoUtil::Conv(polylinePair.second), CBoostGeoUtil::Conv(tmpEdgePairList[0].first));

            createdEdgeList.emplace_back(std::make_pair(polylinePair.first, firstDistance));
            createdEdgeList.emplace_back(std::make_pair(polylinePair.second, secondDistance));
        }

        // 作成したエッジを端から順にソート
        std::sort(createdEdgeList.begin(), createdEdgeList.end(),
            [](std::pair<Boost3DHashPolyline, double>& data1, std::pair<Boost3DHashPolyline, double>& data2)
            {
                return CEpsUtil::Less(data1.second, data2.second);
            });

        // 端から順にペアを作成して保存
        for (int i = 1; i < createdEdgeList.size(); i += 2)
        {
            auto& edge1 = createdEdgeList[i - 1].first;
            auto& edge2 = createdEdgeList[i].first;

            // エッジの方向を調整
            if (bg::distance(edge1.front(), edge2.front()) < bg::distance(edge1.front(), edge2.back()))
            {
                bg::reverse(edge2);
            }

            edgePairList.emplace_back(std::make_pair(edge1, edge2));
        }

        return true;
    }

    // 上記パターンに該当しない場合は失敗
    return false;
}

/*!
 * @brief 隣接道路探索
*/
void CNetworkCreator::SearchNeighborRoad()
{
    CSearchNeighbor searchNeighbor = CSearchNeighbor(m_iLod, m_dLodType);
    searchNeighbor.SetData(this->m_tranRoadData);
}


/*!
 * @brief 道路中心線作成
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::CreateCenterLines()
{
    for (auto tranRoadData : m_tranRoadData)
    {
        for (auto edgePair : tranRoadData->edgePairList)
        {
            ///
            /// 中心線を作成する
            ///
            Boost3DHashPolyline centerLine;
            if (m_iLod == 1)
            {
                if (CreatePolylineBetweenTwoPolylines(edgePair, centerLine) == false)
                {
                    continue;
                }
            }
            else
            {
                if (CreatePolylineBetweenTwoPolylinesUsingParallel(edgePair, centerLine) == false)
                {
                    continue;
                }
            }

            // 作成された中心線を確認
            if (bg::is_empty(centerLine)
                || bg::is_valid(centerLine) == false
                || centerLine.size() < 2)
            {
                continue;
            }

            ///
            /// 中心線の調整
            ///

            Boost3DHashMultiPolygon collisionTargetPolygons;
            switch (m_iLod)
            {
            case 1:
                collisionTargetPolygons.emplace_back(tranRoadData->m_lod1.m_boostGeometry);
                break;
            case 2:
                for (auto lod2 : tranRoadData->m_lod2List)
                {
                    collisionTargetPolygons.emplace_back(lod2.m_boostGeometry);
                }
                break;
            case 3:
                for (auto lod3 : tranRoadData->m_lod3List)
                {
                    collisionTargetPolygons.emplace_back(lod3.m_boostGeometry);
                }
                break;
            default:
                break;
            }

            // 中心線の不足部分を補完する
            auto extendedCenterLine = ExtendPolylineUntilPolygon(centerLine, collisionTargetPolygons);

            // 中心線のはみ出し部分をトリミングする
            auto trimmedCenterLine = TrimPolylineUntilPolygon(extendedCenterLine, collisionTargetPolygons);

            ///
            /// tranRoadDataに中心線を保存
            ///
            CCenterLineData centerLineData(trimmedCenterLine);
            std::shared_ptr<CCenterLineData> ptr = std::make_shared<CCenterLineData>(centerLineData);
            tranRoadData->roadCenterLineList.emplace_back(ptr);

            ///
            /// 中心線のはみ出し確認
            ///
            Boost3DHashMultiLines outsideLines = checkOutsideOfPolygonForRoadway(
                trimmedCenterLine, tranRoadData);
            for (const auto &line : outsideLines)
            {
                Boost3DPointHash pt;
                bg::centroid(line, pt);
                CErrLogger::GetInstance()->WriteRoadwayLog(RoadwayErrType::OUTSIDE_OF_POLYGON, pt);
            }
        }
    }

    return true;
}

/*!
 * @brief 二つのポリラインの間のポリラインを求める
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::CreatePolylineBetweenTwoPolylines(
    std::pair<Boost3DHashPolyline, Boost3DHashPolyline>& inputLinePair,
    Boost3DHashPolyline& outputLine)
{
    double samplingInterval = 1.0; // TODO サンプリング間隔　パラメータ化 or 可変

    // 二つのエッジを取得
    Boost3DHashPolyline firstEdge = inputLinePair.first;
    Boost3DHashPolyline secondEdge = inputLinePair.second;
    Boost3DHashPolyline tmpOutputLine;

    if (bg::is_empty(firstEdge)
        || bg::is_empty(secondEdge))
    {
        return false;
    }

    // 出力ポリラインの初期化
    outputLine.clear();

    // 二つのエッジの長さを測定
    double firstEdgeLength = bg::length(firstEdge);
    double secondEdgeLength = bg::length(secondEdge);

    // 二つのエッジをサンプリング
    // 長いポリラインを指定間隔でサンプリングし、
    // 短いポリラインは長いポリラインのサンプリング個数に合わせる
    Boost3DHashPolyline sampledFirstEdge;
    Boost3DHashPolyline sampledSecondEdge;
    if (CEpsUtil::Less(firstEdgeLength, secondEdgeLength))
    {
        // サンプリングの幅と個数を計算
        sampledSecondEdge = CBoostGeoUtil::Sampling(secondEdge, samplingInterval);
        sampledFirstEdge = CBoostGeoUtil::Sampling(firstEdge, (int)sampledSecondEdge.size());
    }
    else
    {
        // サンプリングの幅と個数を計算
        sampledFirstEdge = CBoostGeoUtil::Sampling(firstEdge, samplingInterval);
        sampledSecondEdge = CBoostGeoUtil::Sampling(secondEdge, (int)sampledFirstEdge.size());
    }

    // インデックスで紐づけ、中心座標を取得
    size_t minSampledSize = 0;
    size_t maxSampledSize = 0;
    size_t halfMinSampledSize = 0;
    if (sampledFirstEdge.size() > sampledSecondEdge.size())
    {
        minSampledSize = sampledSecondEdge.size();
        maxSampledSize = sampledFirstEdge.size();
    }
    else
    {
        minSampledSize = sampledFirstEdge.size();
        maxSampledSize = sampledSecondEdge.size();
    }
    halfMinSampledSize = minSampledSize / 2;

    size_t firstIndexCount = 0;
    size_t secondIndexCount = 0;
    for (int i = 0; i < maxSampledSize; i++)
    {
        Boost3DPointHash firstPoint;
        Boost3DPointHash secondPoint;

        // 中心線の頂点数が異なる場合
        // 頂点数の差分は中心線の中央付近で調整する
        int middleProcessStatus = 0;
        if (maxSampledSize != minSampledSize
            && i >= halfMinSampledSize
            && i < maxSampledSize - halfMinSampledSize)
        {
            if (sampledFirstEdge.size() > sampledSecondEdge.size())
            {
                firstPoint = *(sampledFirstEdge.cbegin() + firstIndexCount);
                secondPoint = *(sampledSecondEdge.crbegin() + secondIndexCount);
                middleProcessStatus = 1;
            }
            else
            {
                firstPoint = *(sampledFirstEdge.cbegin() + firstIndexCount);
                secondPoint = *(sampledSecondEdge.crbegin() + secondIndexCount);
                middleProcessStatus = -1;
            }
        }
        else
        {
            firstPoint = *(sampledFirstEdge.cbegin() + firstIndexCount);
            secondPoint = *(sampledSecondEdge.crbegin() + secondIndexCount);
        }

        // インデックス更新
        if (middleProcessStatus >= 0)
        {
            firstIndexCount++;
        }

        if (middleProcessStatus <= 0)
        {
            secondIndexCount++;
        }

        // 中心座標
        Boost3DPointHash centerPoint = Boost3DPointHash((firstPoint.x() + secondPoint.x()) / 2, (firstPoint.y() + secondPoint.y()) / 2, (firstPoint.z() + secondPoint.z()) / 2);

        // 中心線の座標が連続して同じ場合はスキップ
        if (tmpOutputLine.size() > 0 && tmpOutputLine.back().IsRoundEqual(centerPoint))
        {
            continue;
        }

        // 中心座標からポリライン（中心線）を作成
        tmpOutputLine.emplace_back(centerPoint);
    }

    // 作成された中心線の確認
    if (bg::is_empty(tmpOutputLine)
        || bg::is_valid(tmpOutputLine) == false
        || tmpOutputLine.size() < 2)
    {
        return false;
    }

    // 中心線の間引き
    outputLine = ThinOutVerticesOfPolyline(tmpOutputLine);

    // 作成された中心線の確認
    if (bg::is_empty(outputLine)
        || bg::is_valid(outputLine) == false
        || outputLine.size() < 2)
    {
        return false;
    }

    return true;
}

/*!
 * @brief 二つのポリラインの間のポリラインを求める
 * @brief 両端の平行ではないポリラインは無視する
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::CreatePolylineBetweenTwoPolylinesUsingParallel(
    std::pair<Boost3DHashPolyline, Boost3DHashPolyline>& inputLinePair,
    Boost3DHashPolyline& outputLine)
{
    double samplingInterval = 1.0; // TODO サンプリング間隔　パラメータ化 or 可変
    double footpathCurveRange = 10.0; // TODO 歩道のカーブ検出範囲　パラメータ化 or 可変

    // 二つのエッジを取得
    Boost3DHashPolyline firstEdge = inputLinePair.first;
    Boost3DHashPolyline secondEdge = inputLinePair.second;
    Boost3DHashPolyline tmpOutputLine;

    if (bg::is_empty(firstEdge)
        || bg::is_empty(secondEdge))
    {
        return false;
    }

    // 出力ポリラインの初期化
    outputLine.clear();

    // 二つのエッジの長さを測定
    double firstEdgeLength = bg::length(firstEdge);
    double secondEdgeLength = bg::length(secondEdge);

    // 二つのエッジをサンプリング
    // 長いポリラインを指定間隔でサンプリングし、
    // 短いポリラインは長いポリラインのサンプリング個数に合わせる
    Boost3DHashPolyline sampledFirstEdge;
    Boost3DHashPolyline sampledSecondEdge;
    if (CEpsUtil::Less(firstEdgeLength, secondEdgeLength))
    {
        // サンプリングの幅と個数を計算
        sampledSecondEdge = CBoostGeoUtil::Sampling(secondEdge, samplingInterval);
        sampledFirstEdge = CBoostGeoUtil::Sampling(firstEdge, (int)sampledSecondEdge.size());
    }
    else
    {
        // サンプリングの幅と個数を計算
        sampledFirstEdge = CBoostGeoUtil::Sampling(firstEdge, samplingInterval);
        sampledSecondEdge = CBoostGeoUtil::Sampling(secondEdge, (int)sampledFirstEdge.size());
    }

    // インデックスで紐づけ、中心座標を取得
    size_t minSampledSize = 0;
    size_t maxSampledSize = 0;
    size_t halfMinSampledSize = 0;
    if (sampledFirstEdge.size() > sampledSecondEdge.size())
    {
        minSampledSize = sampledSecondEdge.size();
        maxSampledSize = sampledFirstEdge.size();
    }
    else
    {
        minSampledSize = sampledFirstEdge.size();
        maxSampledSize = sampledSecondEdge.size();
    }
    halfMinSampledSize = minSampledSize / 2;

    // 歩道のカーブ検出用に始点と終点からのポリラインを作成
    Boost3DHashPolyline firstPoint2StartPointPolyline;
    Boost3DHashPolyline firstPoint2EndPointPolyline = sampledFirstEdge;
    Boost3DHashPolyline secondPoint2StartPointPolyline;
    Boost3DHashPolyline secondPoint2EndPointPolyline = sampledSecondEdge;

    size_t firstIndexCount = 0;
    size_t secondIndexCount = 0;
    for (int i = 0; i < maxSampledSize; i++)
    {
        Boost3DPointHash firstPoint;
        Boost3DPointHash secondPoint;

        // 中心線の頂点数が異なる場合
        // 頂点数の差分は中心線の中央付近で調整する
        int middleProcessStatus = 0;
        if (maxSampledSize != minSampledSize
            && i >= halfMinSampledSize
            && i < maxSampledSize - halfMinSampledSize)
        {
            if (sampledFirstEdge.size() > sampledSecondEdge.size())
            {
                firstPoint = *(sampledFirstEdge.cbegin() + firstIndexCount);
                secondPoint = *(sampledSecondEdge.crbegin() + secondIndexCount);
                middleProcessStatus = 1;
            }
            else
            {
                firstPoint = *(sampledFirstEdge.cbegin() + firstIndexCount);
                secondPoint = *(sampledSecondEdge.crbegin() + secondIndexCount);
                middleProcessStatus = -1;
            }
        }
        else
        {
            firstPoint = *(sampledFirstEdge.cbegin() + firstIndexCount);
            secondPoint = *(sampledSecondEdge.crbegin() + secondIndexCount);
        }

        ///
        /// お互いのポリラインが平行かどうか確認する
        ///

        // 平行確認するセグメントの頂点を取得
        Boost3DPointHash firstNextPoint;
        Boost3DPointHash secondNextPoint;
        if (i < halfMinSampledSize)
        {
            firstNextPoint = *(sampledFirstEdge.cbegin() + firstIndexCount + 1);
            secondNextPoint = *(sampledSecondEdge.crbegin() + secondIndexCount + 1);
        }
        else
        {
            firstNextPoint = *(sampledFirstEdge.cbegin() + firstIndexCount - 1);
            secondNextPoint = *(sampledSecondEdge.crbegin() + secondIndexCount - 1);
        }

        // 平行確認するセグメントの方向を取得
        CVector2D firstSegVec = CVector2D(firstNextPoint.x() - firstPoint.x(), firstNextPoint.y() - firstPoint.y());
        CVector2D secondSegVec = CVector2D(secondNextPoint.x() - secondPoint.x(), secondNextPoint.y() - secondPoint.y());

        // インデックス更新
        if (middleProcessStatus >= 0)
        {
            firstPoint2StartPointPolyline.emplace_back(firstPoint);

            if (i > 0
                && firstPoint2EndPointPolyline.size() > 0)
            {
                firstPoint2EndPointPolyline.erase(firstPoint2EndPointPolyline.cbegin());
            }

            firstIndexCount++;
        }
        if (middleProcessStatus <= 0)
        {
            secondPoint2StartPointPolyline.emplace_back(secondPoint);

            if (i > 0
                && secondPoint2EndPointPolyline.size() > 0)
            {
                secondPoint2EndPointPolyline.erase(secondPoint2EndPointPolyline.cend() - 1);
            }

            secondIndexCount++;
        }

        // 各頂点から各ポリラインの始点終点までの距離を求める
        double firstPoint2StartPointLength = bg::length(firstPoint2StartPointPolyline);
        double firstPoint2EndPointLength = bg::length(firstPoint2EndPointPolyline);
        double secondPoint2StartPointLength = bg::length(secondPoint2StartPointPolyline);
        double secondPoint2EndPointLength = bg::length(secondPoint2EndPointPolyline);

        // 始点終点まで一定距離以内の場合、平行確認を行う
        if (firstPoint2StartPointLength < footpathCurveRange
            || firstPoint2EndPointLength < footpathCurveRange
            || secondPoint2StartPointLength < footpathCurveRange
            || secondPoint2EndPointLength < footpathCurveRange)
        {
            // 平行ではない場合、その中心点は保存しない
            if (IsParallel(firstSegVec, secondSegVec, 0) == false)
            {
                continue;
            }
        }

        // 中心座標
        Boost3DPointHash centerPoint = Boost3DPointHash((firstPoint.x() + secondPoint.x()) / 2, (firstPoint.y() + secondPoint.y()) / 2, (firstPoint.z() + secondPoint.z()) / 2);

        // 中心線の座標が連続して同じ場合はスキップ
        if (tmpOutputLine.size() > 0 && tmpOutputLine.back().IsRoundEqual(centerPoint))
        {
            continue;
        }

        // 中心座標からポリライン（中心線）を作成
        tmpOutputLine.emplace_back(centerPoint);
    }

    // 作成された中心線の確認
    if (bg::is_empty(tmpOutputLine)
        || bg::is_valid(tmpOutputLine) == false
        || tmpOutputLine.size() < 2)
    {
        return false;
    }

    // 中心線の間引き
    outputLine = ThinOutVerticesOfPolyline(tmpOutputLine);

    // 作成された中心線の確認
    if (bg::is_empty(outputLine)
        || bg::is_valid(outputLine) == false
        || outputLine.size() < 2)
    {
        return false;
    }

    return true;
}

/*!
 * @brief 車道の幅員計測
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::MeasureRoadWidth()
{
    // tranRoadData毎に処理
    for (auto tranRoadData : m_tranRoadData)
    {
        auto edges = tranRoadData->edgePairList;
        auto& centerLineList = tranRoadData->roadCenterLineList;

        // 道路縁をセグメント毎に始点と方向ベクトルを保存
        std::vector<std::pair<CVector2D, CVector2D>> edgeVecPairList; // エッジの始点と方向ベクトルのペア
        for (auto& edgePair : edges)
        {
            for (int j = 0; j < edgePair.first.size() - 1; j++)
            {
                Boost3DPointHash startPoint = edgePair.first[j];
                Boost3DPointHash endPoint = edgePair.first[j + 1];
                CVector2D edgeVec = CVector2D(endPoint.x() - startPoint.x(), endPoint.y() - startPoint.y());

                edgeVecPairList.emplace_back(std::make_pair(edgeVec, CVector2D(startPoint.x(), startPoint.y())));
            }

            for (int j = 0; j < edgePair.second.size() - 1; j++)
            {
                Boost3DPointHash startPoint = edgePair.second[j];
                Boost3DPointHash endPoint = edgePair.second[j + 1];
                CVector2D edgeVec = CVector2D(endPoint.x() - startPoint.x(), endPoint.y() - startPoint.y());

                edgeVecPairList.emplace_back(std::make_pair(edgeVec, CVector2D(startPoint.x(), startPoint.y())));
            }
        }

        // 中心線毎に計測、保存
        for (auto& centerLineData : centerLineList)
        {
            auto centerLine = &centerLineData->centerLine;
            auto& roadWidth = centerLineData->dMinWidth;
            auto& widthPos = centerLineData->minWidthPos;

            // 中心線の間引き処理実装後、一定間隔で頂点を追加
            auto sampledCenterLine = CBoostGeoUtil::Sampling(*centerLine, 1.0);

            // 中心線のセグメント毎に処理
            for (int i = 0; i < sampledCenterLine.size() - 1; i++)
            {
                /** △     ：firstPoint(baseVec)
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

                Boost3DPointHash firstPoint = sampledCenterLine[i];
                Boost3DPointHash secondPoint = sampledCenterLine[i + 1];

                CVector2D inputVec = CVector2D(secondPoint.x() - firstPoint.x(), secondPoint.y() - firstPoint.y());
                CVector2D rightVec;
                CVector2D leftVec;

                // 注目点(firstPoint)から左右に伸びる垂線ベクトルを求める
                if (CGeoUtil::GetVerticalVec(inputVec, rightVec) == false)
                {
                    continue;
                }

                leftVec = -1 * rightVec;

                CVector2D baseVec = CVector2D(firstPoint.x(), firstPoint.y());

                ///
                /// 左右の交点算出
                ///

                CVector2D rightCrossPoint;
                CVector2D leftCrossPoint;
                double rightCrossLength = LDBL_MAX;
                double leftCrossLength = LDBL_MAX;
                bool isRightCrossPoint = false;
                bool isLeftCrossPoint = false;

                // 保存しなおしたセグメント毎に処理
                for (auto& edgeVecPair : edgeVecPairList)
                {
                    bool isCross = false;
                    CVector2D crossPointTmp;
                    bool isOnLine1 = false;
                    bool isOnLine2 = false;
                    double t = 0.0;
                    double s = 0.0;

                    // 右側交点算出
                    isCross = CGeoUtil::GetCrossPos(rightVec, baseVec, edgeVecPair.first, edgeVecPair.second, crossPointTmp, isOnLine1, isOnLine2, t, s);

                    // 交点の取得成功
                    // かつ、エッジセグメント上に交点がある
                    // かつ、垂直方向に正の向きに交点がある
                    // かつ、交点までの距離がより短い
                    if (isCross == true
                        && isOnLine2 == true
                        && CEpsUtil::Greater(t, 0)
                        && CEpsUtil::Greater(rightCrossLength, t))
                    {
                        // 注目点(firstPoint)から一番近い交点を保存
                        rightCrossLength = t;
                        rightCrossPoint = crossPointTmp;
                        isRightCrossPoint = true;
                    }

                    // 左側交点算出
                    isCross = CGeoUtil::GetCrossPos(leftVec, baseVec, edgeVecPair.first, edgeVecPair.second, crossPointTmp, isOnLine1, isOnLine2, t, s);

                    // 交点の取得成功
                    // かつ、エッジセグメント上に交点がある
                    // かつ、垂直方向に正の向きに交点がある
                    // かつ、交点までの距離がより短い
                    if (isCross == true
                        && isOnLine2 == true
                        && CEpsUtil::Greater(t, 0)
                        && CEpsUtil::Greater(leftCrossLength, t))
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
                // この中心線の幅員とその位置を更新する
                if(CEpsUtil::LessEqual(roadWidth, 0.0) || CEpsUtil::Greater(roadWidth, segmentWidth))
                {
                    roadWidth = segmentWidth;
                    widthPos = firstPoint;
                }
            }
        }
    }

    return true;
}

/*!
 * @brief 交差点接続
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::ConnectIntersection()
{
    // スキップ道路リスト
    // 隣接道路が交差点ですでに交差点接続しているものはスキップ
    std::set<std::shared_ptr<CTranRoadData>> exclusionRoadList;

    for (auto tranRoadDataPtr : m_tranRoadData)
    {
        // 交差点探索
        // 隣接道路が3つ未満なら交差点ではないため除外

        // LOD1は種別がないためtrue固定にする
        bool isContainIntersection = false;
        switch (m_iLod)
        {
        case 1:
            isContainIntersection = true;
            break;
        case 2:
            for (auto lod2 : tranRoadDataPtr->m_lod2List)
            {
                if (lod2.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION)
                {
                    isContainIntersection = true;
                    break;
                }
            }
            break;
        case 3:
            for (auto lod3 : tranRoadDataPtr->m_lod3List)
            {
                if (lod3.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION)
                {
                    isContainIntersection = true;
                    break;
                }
            }
            break;
        default:
            break;
        }

        if (isContainIntersection == false || tranRoadDataPtr->m_nInOut < 3)
        {
            continue;
        }

        // スキップ道路リストに含まれていたらスキップ
        if (exclusionRoadList.find(tranRoadDataPtr) != exclusionRoadList.cend())
        {
            continue;
        }

        ///
        /// 処理対象のデータを取得
        /// 交差部ポリゴン
        /// 隣接する車道ポリゴン
        ///

        // 隣接道路が交差部の場合
        // その隣接道路もまとめて交差点接続する
        std::set<std::shared_ptr<CTranRoadData>> neighborRoadPtrList;
        std::set<std::shared_ptr<CTranRoadData>> intersectionRoadList;
        if (GetNeighborRoadOfIntersection(tranRoadDataPtr, neighborRoadPtrList, intersectionRoadList) == false)
        {
            continue;
        }

        // 隣接する交差点はスキップ道路リストに追加
        for (auto intersection : intersectionRoadList)
        {
            exclusionRoadList.insert(intersection);
        }

        // 隣接車道部のジオメトリを取得
        Boost3DHashMultiPolygon neighborRoadPolygons;
        switch (m_iLod)
        {
        case 1:
            for (auto neighborRoadPtr : neighborRoadPtrList)
            {
                neighborRoadPolygons.emplace_back(neighborRoadPtr->m_lod1.m_boostGeometry);
            }
            break;
        case 2:
            for (auto neighborRoadPtr : neighborRoadPtrList)
            {
                for (auto lod2 : neighborRoadPtr->m_lod2List)
                {
                    if (lod2.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY
                        || lod2.m_fuctionType == (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND)
                    {
                        neighborRoadPolygons.emplace_back(lod2.m_boostGeometry);
                    }
                }
            }
            break;
        case 3:
            for (auto neighborRoadPtr : neighborRoadPtrList)
            {
                for (auto lod3 : neighborRoadPtr->m_lod3List)
                {
                    if (lod3.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY
                        || lod3.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::LANE
                        || lod3.m_fuctionType == (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND)
                    {
                        neighborRoadPolygons.emplace_back(lod3.m_boostGeometry);
                    }
                }
            }
            break;
        default:
            break;
        }

        // 交差部のジオメトリを取得
        // 併せて、交差部の島のみジオメトリを取得
        Boost3DHashMultiPolygon intersectionPolygons;
        Boost3DHashMultiPolygon islandOnlyPolygons;
        switch (m_iLod)
        {
        case 1:
            for (auto intersectionPtr : intersectionRoadList)
            {
                intersectionPolygons.emplace_back(intersectionPtr->m_lod1.m_boostGeometry);
            }
            break;
        case 2:
            for (auto intersectionPtr : intersectionRoadList)
            {
                for (auto lod2 : intersectionPtr->m_lod2List)
                {
                    if (lod2.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY
                        || lod2.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION
                        || lod2.m_fuctionType == (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND)
                    {
                        intersectionPolygons.emplace_back(lod2.m_boostGeometry);
                    }

                    if (lod2.m_fuctionType == (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND)
                    {
                        islandOnlyPolygons.emplace_back(lod2.m_boostGeometry);
                    }

                    if (lod2.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH)
                    {
                        // 島と隣接している歩道は島と同等の処理をする
                        bool isNeighborIsland = false;
                        for (auto tmpLod2 : intersectionPtr->m_lod2List)
                        {
                            double dist = bg::distance(CBoostGeoUtil::Conv(lod2.m_boostGeometry), CBoostGeoUtil::Conv(tmpLod2.m_boostGeometry));

                            if (CEpsUtil::Equal(dist, 0.0)
                                && tmpLod2.m_fuctionType == (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND)
                            {
                                isNeighborIsland = true;
                                break;
                            }
                        }
                        if (isNeighborIsland)
                        {
                            intersectionPolygons.emplace_back(lod2.m_boostGeometry);
                            islandOnlyPolygons.emplace_back(lod2.m_boostGeometry);
                        }
                    }
                }
            }
            break;
        case 3:
            for (auto intersectionPtr : intersectionRoadList)
            {
                for (auto lod3 : intersectionPtr->m_lod3List)
                {
                    if (lod3.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY
                        || lod3.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::LANE
                        || lod3.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION
                        || lod3.m_fuctionType == (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND)
                    {
                        intersectionPolygons.emplace_back(lod3.m_boostGeometry);
                    }

                    if (lod3.m_fuctionType == (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND)
                    {
                        islandOnlyPolygons.emplace_back(lod3.m_boostGeometry);
                    }

                    if (lod3.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH)
                    {
                        // 島と隣接している歩道は島と同等の処理をする
                        bool isNeighborIsland = false;
                        for (auto tmpLod3 : intersectionPtr->m_lod3List)
                        {
                            double dist = bg::distance(CBoostGeoUtil::Conv(lod3.m_boostGeometry), CBoostGeoUtil::Conv(tmpLod3.m_boostGeometry));

                            if (CEpsUtil::Equal(dist, 0.0)
                                && tmpLod3.m_fuctionType == (int)AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND)
                            {
                                isNeighborIsland = true;
                                break;
                            }
                        }
                        if (isNeighborIsland)
                        {
                            intersectionPolygons.emplace_back(lod3.m_boostGeometry);
                            islandOnlyPolygons.emplace_back(lod3.m_boostGeometry);
                        }
                    }
                }
            }
            break;
        default:
            break;
        }

        // 対象のtranRoadDataに交差点のポリゴンがなかった場合はスキップ
        if (intersectionPolygons.empty())
        {
            continue;
        }

        // この交差点の重心を取得
        // （交差点との距離を測定するために使用）
        Boost3DPointHash centroidIntersection;
        bg::centroid(intersectionPolygons, centroidIntersection);

        ///
        /// 隣接道路の他データの取得
        /// 隣接道路を幅員が大きい順に整理
        ///

        std::vector<NeighborRoadInfo> neighborRoadInfoList;
        if (GetNeighborRoadInfoList(neighborRoadPtrList, intersectionPolygons, neighborRoadInfoList) == false)
        {
            continue;
        }

        // 交差点内の中心線
        // すべての接続が完了したら隣接道路の中心線を延長する
        Boost3DHashMultiLines createdIntersectionLines;

        ///
        /// メイン道路を決定する
        ///
        if (ConnectMainCenterLine(intersectionPolygons, neighborRoadPolygons, neighborRoadInfoList, createdIntersectionLines) == false)
        {
            continue;
        }

        ///
        /// 残りの道路をつなげる
        ///
        if (ConnectSubCenterLine(intersectionPolygons, islandOnlyPolygons, neighborRoadPolygons, neighborRoadInfoList, createdIntersectionLines) == false)
        {
            continue;
        }

        ///
        /// メイン道路が複数の場合に
        /// 隣接道路の中心線を基に
        /// メイン道路を接続しなおす
        ///
        for (auto& info : neighborRoadInfoList)
        {
            if (info.RoadCount < 2)
            {
                continue;
            }

            // メイン道路が複数の場合に
            // メイン道路を接続しなおす
            for (auto& line : createdIntersectionLines)
            {
                if (bg::is_empty(line))
                {
                    continue;
                }

                if (line.front().IsRoundEqual(info.ConnectPoint))
                {
                    // メイン中心線の先端を削除
                    line.erase(line.cbegin());

                    auto additionalCrossPoint = line.front();

                    // 先端を代わりに隣接道路情報の子道路の中心線につなげる
                    for (int i = 0; i < info.ChildNeighborRoadInfo.size(); i++)
                    {
                        auto& childInfo = info.ChildNeighborRoadInfo[i];

                        // 中心線が作成できなかったものはスキップ
                        if (childInfo.IsConnectedIntersection == false)
                        {
                            continue;
                        }

                        if (i == 0)
                        {
                            // 最初の子道路は元の中心線を延伸する
                            line.insert(line.cbegin(), childInfo.ConnectPoint);
                        }
                        else
                        {
                            // 2つ目以降の子道路は中心線を新しく追加する
                            Boost3DHashPolyline additionalMainCenterLine;
                            additionalMainCenterLine.emplace_back(childInfo.ConnectPoint);
                            additionalMainCenterLine.emplace_back(additionalCrossPoint);
                            createdIntersectionLines.emplace_back(additionalMainCenterLine);
                        }
                    }
                }
                else if (line.back().IsRoundEqual(info.ConnectPoint))
                {
                    // メイン中心線の先端を削除
                    line.erase(line.cend() - 1);

                    auto additionalCrossPoint = line.back();

                    // 先端を代わりに隣接道路情報の子道路の中心線につなげる
                    for (int i = 0; i < info.ChildNeighborRoadInfo.size(); i++)
                    {
                        auto& childInfo = info.ChildNeighborRoadInfo[i];

                        // 中心線が作成できなかったものはスキップ
                        if (childInfo.IsConnectedIntersection == false)
                        {
                            continue;
                        }

                        if (i == 0)
                        {
                            // 最初の子道路は元の中心線を延伸する
                            line.insert(line.cend(), childInfo.ConnectPoint);
                        }
                        else
                        {
                            // 2つ目以降の子道路は中心線を新しく追加する
                            Boost3DHashPolyline additionalMainCenterLine;
                            additionalMainCenterLine.emplace_back(childInfo.ConnectPoint);
                            additionalMainCenterLine.emplace_back(additionalCrossPoint);
                            createdIntersectionLines.emplace_back(additionalMainCenterLine);
                        }
                    }
                }
            }
        }

        ///
        /// 作成した中心線を交差部に保存する
        ///
        for (auto intersectionCenterLine : createdIntersectionLines)
        {
            CCenterLineData centerLineData(intersectionCenterLine);

            // 属性を追加する
            // 交差点は幅員や勾配等の情報は無く
            // 交差点フラグのみ設定
            centerLineData.isIntersection = true;

            std::shared_ptr<CCenterLineData> ptr = std::make_shared<CCenterLineData>(centerLineData);
            tranRoadDataPtr->roadCenterLineList.emplace_back(ptr);
        }

        // 中心線がひとつでもあれば
        // 周辺の交差部の中心線ありフラグを設定する
        if (createdIntersectionLines.size() > 0)
        {
            // 周辺の交差部を総検索
            for (auto intersectionRoadPtr : intersectionRoadList)
            {
                // この交差部はフラグ設定しない
                if (intersectionRoadPtr == tranRoadDataPtr)
                {
                    continue;
                }

                // フラグ設定
                intersectionRoadPtr->m_bIsCenterLineOnNeighborRoad = true;
            }
        }
    }

    return true;
}

/*!
 * @brief 交差点の隣接道路を取得
 * @param[in ] tranRoadData             対象道路
 * @param[out] neighborRoadList         対象道路（隣接交差部を含む）に隣接している車道部リスト
 * @param[out] intersectionRoadList     対象道路に隣接している交差部リスト
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::GetNeighborRoadOfIntersection(std::shared_ptr<CTranRoadData>& tranRoadData, std::set<std::shared_ptr<CTranRoadData>>& neighborRoadList, std::set<std::shared_ptr<CTranRoadData>>& intersectionRoadList)
{
    if (tranRoadData == nullptr)
    {
        return false;
    }

    // 対象道路を交差部リストに追加する
    intersectionRoadList.insert(tranRoadData);

    for (auto neighborRoadPtr : tranRoadData->m_neighborRoadPtr)
    {
        // すでに車道部リストか交差部リストに追加済みの場合はスキップ
        auto neighborRoadListFindIt = neighborRoadList.find(neighborRoadPtr);
        auto intersectionRoadListFindIt = intersectionRoadList.find(neighborRoadPtr);
        if (neighborRoadListFindIt != neighborRoadList.cend()
            || intersectionRoadListFindIt != intersectionRoadList.cend())
        {
            continue;
        }

        // 交差部のポリゴンが含まれているか確認
        // LOD1は種別がないためtrue固定にする
        bool isContainIntersection = false;

        switch (m_iLod)
        {
        case 1:
            isContainIntersection = true;
            break;
        case 2:
            for (auto lod2 : neighborRoadPtr->m_lod2List)
            {
                if (lod2.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION)
                {
                    isContainIntersection = true;
                    break;
                }
            }
            break;
        case 3:
            for (auto lod3 : neighborRoadPtr->m_lod3List)
            {
                if (lod3.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION)
                {
                    isContainIntersection = true;
                    break;
                }
            }
            break;
        default:
            break;
        }

        if ((isContainIntersection || neighborRoadPtr->m_neighborRoadPtr.size() > 2)
            && neighborRoadPtr->roadCenterLineList.size() == 0)
        {
            // 交差部の場合

            // この交差部を対象道路として隣接道路を取得
            if (GetNeighborRoadOfIntersection(neighborRoadPtr, neighborRoadList, intersectionRoadList) == false)
            {
                return false;
            }
        }
        else
        {
            // 車道部の場合

            // 車道部リストに保存する
            neighborRoadList.insert(neighborRoadPtr);
        }
    }

    return true;
}

/*!
 * @brief 交差点の隣接道路の情報を取得する
 * @param[in ] neighborRoadPtrList      交差部に隣接している道路情報リスト
 * @param[in ] intersectionPolygonList  交差部の車道部ポリゴンリスト
 * @param[out] neighborRoadInfoList     隣接道路情報リスト
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::GetNeighborRoadInfoList(
    std::set<std::shared_ptr<CTranRoadData>>& neighborRoadPtrList,
    Boost3DHashMultiPolygon& intersectionPolygonList,
    std::vector<NeighborRoadInfo>& neighborRoadInfoList)
{
    neighborRoadInfoList.clear();

    for (auto neighborRoadPtr : neighborRoadPtrList)
    {
        std::vector<NeighborRoadInfo> tmpNeighborRoadInfoList;

        for (auto centerLineData : neighborRoadPtr->roadCenterLineList)
        {
            // 中心線取得
            auto centerLinePtr = &centerLineData->centerLine;

            // 中心線の両端の点を取得
            Boost3DPointHash startCenterLinePoint = (*centerLinePtr)[0];
            Boost3DPointHash endCenterLinePoint = (*centerLinePtr)[centerLinePtr->size() - 1];

            // 交差部との距離を算出
            double DistStart2Intersection = bg::distance(startCenterLinePoint, intersectionPolygonList);
            double DistEnd2Intersection = bg::distance(endCenterLinePoint, intersectionPolygonList);

            // 交差部と近いほうの点を保存する
            if (CEpsUtil::Greater(DistStart2Intersection, DistEnd2Intersection))
            {
                // 中心線の方向ベクトル
                CVector2D point1 = CVector2D((*centerLinePtr)[centerLinePtr->size() - 1].x(), (*centerLinePtr)[centerLinePtr->size() - 1].y());
                CVector2D point2 = CVector2D((*centerLinePtr)[centerLinePtr->size() - 2].x(), (*centerLinePtr)[centerLinePtr->size() - 2].y());
                CVector2D centerLineVector = point1 - point2;

                tmpNeighborRoadInfoList.emplace_back(NeighborRoadInfo(endCenterLinePoint, centerLineVector, centerLineData->dMinWidth, centerLineData));
            }
            else
            {
                // 中心線の方向ベクトル
                CVector2D point1 = CVector2D((*centerLinePtr)[0].x(), (*centerLinePtr)[0].y());
                CVector2D point2 = CVector2D((*centerLinePtr)[1].x(), (*centerLinePtr)[1].y());
                CVector2D centerLineVector = point1 - point2;

                tmpNeighborRoadInfoList.emplace_back(NeighborRoadInfo(startCenterLinePoint, centerLineVector, centerLineData->dMinWidth, centerLineData));
            }
        }

        if (tmpNeighborRoadInfoList.size() == 0)
        {
            continue;
        }
        else if (tmpNeighborRoadInfoList.size() == 1)
        {
            neighborRoadInfoList.emplace_back(NeighborRoadInfo(tmpNeighborRoadInfoList.front().ConnectPoint, tmpNeighborRoadInfoList.front().ConnectVector2D, tmpNeighborRoadInfoList.front().Width, tmpNeighborRoadInfoList.front().CenterLinePtr));
        }
        else
        {
            neighborRoadInfoList.emplace_back(NeighborRoadInfo(tmpNeighborRoadInfoList));
        }
    }

    if (neighborRoadInfoList.size() < 1)
    {
        return false;
    }

    // 幅員順にソート
    std::sort(neighborRoadInfoList.begin(), neighborRoadInfoList.end(), [](const NeighborRoadInfo& info1, const NeighborRoadInfo& info2){ return CEpsUtil::Greater(info1.Width, info2.Width); });

    return true;
}

/*!
 * @brief 交差点の主道路を接続する
 * @param[in ] intersectionPolygonList      交差部ポリゴンリスト
 * @param[in ] neighborRoadPolygonList      隣接道路の車道部ポリゴンリスト
 * @param[in ] neighborRoadInfoList         隣接道路情報リスト
 * @param[out] intersectionPolylines        交差点内中心線
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::ConnectMainCenterLine(
    Boost3DHashMultiPolygon& intersectionPolygonList,
    Boost3DHashMultiPolygon& neighborRoadPolygonList,
    std::vector<NeighborRoadInfo>& neighborRoadInfoList,
    Boost3DHashMultiLines& intersectionPolylines)
{
    ///
    /// 隣接道路のサイズ確認
    ///

    if (neighborRoadInfoList.size() < 1)
    {
        return false;
    }

    if (neighborRoadInfoList.size() == 1)
    {
        Boost3DPointHash centroidPoint;
        bg::centroid(intersectionPolygonList, centroidPoint);

        Boost3DHashPolyline insertCenterLine;
        insertCenterLine.emplace_back(neighborRoadInfoList.front().ConnectPoint);
        insertCenterLine.emplace_back(centroidPoint);
        intersectionPolylines.emplace_back(insertCenterLine);

        return true;
    }

    ///
    /// 主道路二つを決定する
    ///

    NeighborRoadInfo* firstNeighborRoadInfo = nullptr;
    NeighborRoadInfo* secondNeighborRoadInfo = nullptr;
    bool isDetectedParallel = false;

    // 二車線以上の道路が2つ以上ある場合は
    // 二車線以上の道路が主道路になるかどうか確認する
    int multiCenterLineCount = 0;
    for (auto info : neighborRoadInfoList)
    {
        if (info.RoadCount > 1)
        {
            multiCenterLineCount++;
        }
    }

    if (multiCenterLineCount > 1)
    {
        // 一つ目の隣接道路を検索(太い道路から順に検索する)
        for (int i = 0; i < neighborRoadInfoList.size(); i++)
        {
            if (neighborRoadInfoList[i].RoadCount < 2)
            {
                continue;
            }

            firstNeighborRoadInfo = &(neighborRoadInfoList[i]);

            // 一つ目の隣接道路を検索
            for (int j = i + 1; j < neighborRoadInfoList.size(); j++)
            {
                if (neighborRoadInfoList[j].RoadCount < 2)
                {
                    continue;
                }

                secondNeighborRoadInfo = &(neighborRoadInfoList[j]);

                if (IsParallel(firstNeighborRoadInfo->ConnectVector2D, secondNeighborRoadInfo->ConnectVector2D, -1))
                {
                    isDetectedParallel = true;
                    break;
                }
            }

            if (isDetectedParallel)
            {
                break;
            }
        }
    }

    // 二車線以上の道路の中で主道路が見つからなかったら
    // 一車線道路も含めて検索する

    // 平行な道路を検索
    if (isDetectedParallel == false)
    {
        // 一つ目の隣接道路を検索(太い道路から順に検索する)
        for (int i = 0; i < neighborRoadInfoList.size(); i++)
        {
            firstNeighborRoadInfo = &(neighborRoadInfoList[i]);

            // 一つ目の隣接道路を検索
            for (int j = i + 1; j < neighborRoadInfoList.size(); j++)
            {
                secondNeighborRoadInfo = &(neighborRoadInfoList[j]);

                if (IsParallel(firstNeighborRoadInfo->ConnectVector2D, secondNeighborRoadInfo->ConnectVector2D, -1))
                {
                    isDetectedParallel = true;
                    break;
                }
            }

            if (isDetectedParallel)
            {
                break;
            }
        }
    }

    // 平行な道路がなかった場合
    // 一番太い道路と二番目に太い道路を結ぶ
    if (isDetectedParallel == false)
    {
        firstNeighborRoadInfo = &(neighborRoadInfoList[0]);
        secondNeighborRoadInfo = &(neighborRoadInfoList[1]);
    }

    ///
    /// 主道路交点の内外判定用ポリゴンの作成
    ///

    Boost3DHashMultiPolygon neighborAndIntersectionPolygon;

    // 交差部と現在の隣接道路の合同ポリゴン
    // 交差点内外判定に使用
    Boost3DHashMultiPolygon tmpPolygons;

    for (auto intersection : intersectionPolygonList)
    {
        tmpPolygons.emplace_back(intersection);
    }
    for (auto neighborRoad : neighborRoadPolygonList)
    {
        tmpPolygons.emplace_back(neighborRoad);
    }

    // 融合可能であれば融合する
    if (tmpPolygons.size() > 1)
    {
        neighborAndIntersectionPolygon = CGDALUtil::GetInstance()->Dissolve(tmpPolygons);
    }
    else
    {
        neighborAndIntersectionPolygon = tmpPolygons;
    }

    ///
    /// 主道路の交点を決定する
    ///

    Boost3DPointHash middlePoint;
    Boost3DMultiPointHashs middlePointList;
    bool isDetectedCrossPoint = false;

    // 隣接道路から交差点までのポリライン
    // 交差点内外判定に使用
    Boost3DHashPolyline neighbor2IntersectionPolyline;
    Boost3DHashMultiLines neighbor2IntersectionPolylineList;

    // 2つの主道路の車線数が同じかどうか
    bool isSameMainCenterLineCount = firstNeighborRoadInfo->RoadCount == secondNeighborRoadInfo->RoadCount;

    if (isSameMainCenterLineCount
        && firstNeighborRoadInfo->RoadCount > 1)
    {
        // 同じ車線数の場合は車線の数だけ中心線、中間点を作成する

        Boost3DMultiPointHashs firstNeighborRoadCrossPointList;
        Boost3DMultiPointHashs secondNeighborRoadCrossPointList;
        std::vector<std::pair<size_t, size_t>> pairingCenterLineIndexList;

        for (auto childInfo : firstNeighborRoadInfo->ChildNeighborRoadInfo)
        {
            firstNeighborRoadCrossPointList.emplace_back(childInfo.ConnectPoint);
        }
        for (auto childInfo : secondNeighborRoadInfo->ChildNeighborRoadInfo)
        {
            secondNeighborRoadCrossPointList.emplace_back(childInfo.ConnectPoint);
        }

        // 車線のペアリング
        if (PairMultiplePoints(
            firstNeighborRoadCrossPointList,
            secondNeighborRoadCrossPointList,
            pairingCenterLineIndexList))
        {
            // 中間点
            for (auto indexPair : pairingCenterLineIndexList)
            {
                middlePoint = Boost3DPointHash(
                    (firstNeighborRoadInfo->ChildNeighborRoadInfo[indexPair.first].ConnectPoint.x() + secondNeighborRoadInfo->ChildNeighborRoadInfo[indexPair.second].ConnectPoint.x()) / 2,
                    (firstNeighborRoadInfo->ChildNeighborRoadInfo[indexPair.first].ConnectPoint.y() + secondNeighborRoadInfo->ChildNeighborRoadInfo[indexPair.second].ConnectPoint.y()) / 2,
                    (firstNeighborRoadInfo->ChildNeighborRoadInfo[indexPair.first].ConnectPoint.z() + secondNeighborRoadInfo->ChildNeighborRoadInfo[indexPair.second].ConnectPoint.z()) / 2);

                neighbor2IntersectionPolyline.clear();
                neighbor2IntersectionPolyline.emplace_back(firstNeighborRoadInfo->ChildNeighborRoadInfo[indexPair.first].ConnectPoint);
                neighbor2IntersectionPolyline.emplace_back(middlePoint);
                neighbor2IntersectionPolyline.emplace_back(secondNeighborRoadInfo->ChildNeighborRoadInfo[indexPair.second].ConnectPoint);

                middlePointList.emplace_back(middlePoint);
                neighbor2IntersectionPolylineList.emplace_back(neighbor2IntersectionPolyline);
            }

            // 結んだ点が交差点のポリゴン内にあるか確認
            isDetectedCrossPoint = CoveredByPolygonsIgnoreZ(middlePointList, intersectionPolygonList)
                && CheckPolylineProtrudeFromPolygon(neighbor2IntersectionPolylineList, neighborAndIntersectionPolygon) == false;

            if (isDetectedCrossPoint == false)
            {
                middlePointList.clear();
                neighbor2IntersectionPolyline.clear();
                for (auto polyline : neighbor2IntersectionPolylineList)  polyline.clear();
                neighbor2IntersectionPolylineList.clear();

                // 中心線を延長させた交点
                for (auto indexPair : pairingCenterLineIndexList)
                {
                    bool isLine1 = false;
                    bool isLine2 = false;
                    double t = 0.0;
                    double s = 0.0;
                    CVector2D crossPoint;
                    if (CGeoUtil::GetCrossPos(
                        firstNeighborRoadInfo->ConnectVector2D,
                        CVector2D(firstNeighborRoadInfo->ChildNeighborRoadInfo[indexPair.first].ConnectPoint.x(), firstNeighborRoadInfo->ChildNeighborRoadInfo[indexPair.first].ConnectPoint.y()),
                        secondNeighborRoadInfo->ConnectVector2D,
                        CVector2D(secondNeighborRoadInfo->ChildNeighborRoadInfo[indexPair.second].ConnectPoint.x(), secondNeighborRoadInfo->ChildNeighborRoadInfo[indexPair.second].ConnectPoint.y()),
                        crossPoint,
                        isLine1,
                        isLine2,
                        t,
                        s
                    ))
                    {
                        middlePoint = Boost3DPointHash(
                            crossPoint.x,
                            crossPoint.y,
                            (firstNeighborRoadInfo->ChildNeighborRoadInfo[indexPair.first].ConnectPoint.z() + secondNeighborRoadInfo->ChildNeighborRoadInfo[indexPair.second].ConnectPoint.z()) / 2); // TODO 詳しい高さはLOD3の三角ポリゴンを取得して求める

                        neighbor2IntersectionPolyline.clear();
                        neighbor2IntersectionPolyline.emplace_back(firstNeighborRoadInfo->ChildNeighborRoadInfo[indexPair.first].ConnectPoint);
                        neighbor2IntersectionPolyline.emplace_back(middlePoint);
                        neighbor2IntersectionPolyline.emplace_back(secondNeighborRoadInfo->ChildNeighborRoadInfo[indexPair.second].ConnectPoint);

                        middlePointList.emplace_back(middlePoint);
                        neighbor2IntersectionPolylineList.emplace_back(neighbor2IntersectionPolyline);
                    }
                }

                // 結んだ点が交差点のポリゴン内にあるか確認
                isDetectedCrossPoint = CoveredByPolygonsIgnoreZ(middlePointList, intersectionPolygonList)
                    && CheckPolylineProtrudeFromPolygon(neighbor2IntersectionPolylineList, neighborAndIntersectionPolygon) == false;
            }

            // 重心は1点にまとまってしまうため
            // ここでは処理しない
        }
    }

    // 車線数が異なる場合は一つの点にまとめる形で中心線、中間点を作成する

    // 中間点
    if (isDetectedCrossPoint == false)
    {
        middlePointList.clear();
        neighbor2IntersectionPolyline.clear();
        for (auto polyline : neighbor2IntersectionPolylineList)  polyline.clear();
        neighbor2IntersectionPolylineList.clear();

        middlePoint = Boost3DPointHash(
            (firstNeighborRoadInfo->ConnectPoint.x() + secondNeighborRoadInfo->ConnectPoint.x()) / 2,
            (firstNeighborRoadInfo->ConnectPoint.y() + secondNeighborRoadInfo->ConnectPoint.y()) / 2,
            (firstNeighborRoadInfo->ConnectPoint.z() + secondNeighborRoadInfo->ConnectPoint.z()) / 2);

        neighbor2IntersectionPolyline.emplace_back(firstNeighborRoadInfo->ConnectPoint);
        neighbor2IntersectionPolyline.emplace_back(middlePoint);
        neighbor2IntersectionPolyline.emplace_back(secondNeighborRoadInfo->ConnectPoint);

        middlePointList.emplace_back(middlePoint);
        neighbor2IntersectionPolylineList.emplace_back(neighbor2IntersectionPolyline);

        // 結んだ点が交差点のポリゴン内にあるか確認
        isDetectedCrossPoint = CoveredByPolygonsIgnoreZ(middlePointList, intersectionPolygonList)
            && CheckPolylineProtrudeFromPolygon(neighbor2IntersectionPolylineList, neighborAndIntersectionPolygon) == false;
    }

    if (isDetectedCrossPoint == false)
    {
        middlePointList.clear();
        neighbor2IntersectionPolyline.clear();
        for (auto polyline : neighbor2IntersectionPolylineList)  polyline.clear();
        neighbor2IntersectionPolylineList.clear();

        // 中心線を延長させた交点
        bool isLine1 = false;
        bool isLine2 = false;
        double t = 0.0;
        double s = 0.0;
        CVector2D crossPoint;
        if (CGeoUtil::GetCrossPos(
            firstNeighborRoadInfo->ConnectVector2D,
            CVector2D(firstNeighborRoadInfo->ConnectPoint.x(), firstNeighborRoadInfo->ConnectPoint.y()),
            secondNeighborRoadInfo->ConnectVector2D,
            CVector2D(secondNeighborRoadInfo->ConnectPoint.x(), secondNeighborRoadInfo->ConnectPoint.y()),
            crossPoint,
            isLine1,
            isLine2,
            t,
            s
        ))
        {
            middlePoint = Boost3DPointHash(
                crossPoint.x,
                crossPoint.y,
                (firstNeighborRoadInfo->ConnectPoint.z() + secondNeighborRoadInfo->ConnectPoint.z()) / 2); // TODO 詳しい高さはLOD3の三角ポリゴンを取得して求める

            neighbor2IntersectionPolyline.emplace_back(firstNeighborRoadInfo->ConnectPoint);
            neighbor2IntersectionPolyline.emplace_back(middlePoint);
            neighbor2IntersectionPolyline.emplace_back(secondNeighborRoadInfo->ConnectPoint);

            middlePointList.emplace_back(middlePoint);
            neighbor2IntersectionPolylineList.emplace_back(neighbor2IntersectionPolyline);

            // 結んだ点が交差点のポリゴン内にあるか確認
            isDetectedCrossPoint = CoveredByPolygonsIgnoreZ(middlePointList, intersectionPolygonList)
                && CheckPolylineProtrudeFromPolygon(neighbor2IntersectionPolylineList, neighborAndIntersectionPolygon) == false;
        }
    }

    if (isDetectedCrossPoint == false)
    {
        middlePointList.clear();
        neighbor2IntersectionPolyline.clear();
        for (auto polyline : neighbor2IntersectionPolylineList)  polyline.clear();
        neighbor2IntersectionPolylineList.clear();

        // 重心
        bg::centroid(intersectionPolygonList, middlePoint);

        neighbor2IntersectionPolyline.emplace_back(firstNeighborRoadInfo->ConnectPoint);
        neighbor2IntersectionPolyline.emplace_back(middlePoint);
        neighbor2IntersectionPolyline.emplace_back(secondNeighborRoadInfo->ConnectPoint);

        middlePointList.emplace_back(middlePoint);
        neighbor2IntersectionPolylineList.emplace_back(neighbor2IntersectionPolyline);

        // 結んだ点が交差点のポリゴン内にあるか確認
        isDetectedCrossPoint = CoveredByPolygonsIgnoreZ(middlePointList, intersectionPolygonList)
            && CheckPolylineProtrudeFromPolygon(neighbor2IntersectionPolylineList, neighborAndIntersectionPolygon) == false;
    }

    if (isDetectedCrossPoint == false)
    {
        // 重心点でもポリゴン外にはみ出す場合は、
        // この交差点の接続は行わない

        return false;
    }

    ///
    /// 結果を保存する
    ///
    for(auto polyline : neighbor2IntersectionPolylineList)  intersectionPolylines.emplace_back(polyline);

    if (intersectionPolylines.size() < 1)
    {
        return false;
    }

    firstNeighborRoadInfo->IsConnectedIntersection = true;

    secondNeighborRoadInfo->IsConnectedIntersection = true;

    if (firstNeighborRoadInfo->RoadCount > 1)
    {
        for (auto& item : firstNeighborRoadInfo->ChildNeighborRoadInfo)
        {
            item.IsConnectedIntersection = true;
        }
    }

    if (secondNeighborRoadInfo->RoadCount > 1)
    {
        for (auto& item : secondNeighborRoadInfo->ChildNeighborRoadInfo)
        {
            item.IsConnectedIntersection = true;
        }
    }

    return true;
}

/*!
 * @brief 交差点の従道路を接続する
 * @param[in    ] intersectionPolygonList               交差部ポリゴンリスト
 * @param[in    ] intersectionIslandOnlyPolygonList     交差部の島ポリゴンリスト
 * @param[in    ] neighborRoadPolygonList               隣接道路の車道部ポリゴンリスト
 * @param[in    ] neighborRoadInfoList                  隣接道路情報リスト
 * @param[in,out] intersectionPolylines                 交差点内中心線
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::ConnectSubCenterLine(
    Boost3DHashMultiPolygon& intersectionPolygonList,
    Boost3DHashMultiPolygon& intersectionIslandOnlyPolygonList,
    Boost3DHashMultiPolygon& neighborRoadPolygonList,
    std::vector<NeighborRoadInfo>& neighborRoadInfoList,
    Boost3DHashMultiLines& intersectionPolylines)
{
    ///
    /// 隣接道路のサイズ確認
    ///

    if (neighborRoadInfoList.size() < 1)
    {
        return false;
    }

    if (neighborRoadInfoList.size() < 3)
    {
        // 主道路の接続のみのため終了
        return true;
    }

    ///
    /// 主道路交点の内外判定用ポリゴンの作成
    ///

    Boost3DHashMultiPolygon neighborAndIntersectionPolygon;

    // 交差部と現在の隣接道路の合同ポリゴン
    // 交差点内外判定に使用
    Boost3DHashMultiPolygon tmpPolygons;

    for (auto intersection : intersectionPolygonList)
    {
        tmpPolygons.emplace_back(intersection);
    }
    for (auto neighborRoad : neighborRoadPolygonList)
    {
        tmpPolygons.emplace_back(neighborRoad);
    }

    // 融合可能であれば融合する
    if (tmpPolygons.size() > 1)
    {
        neighborAndIntersectionPolygon = CGDALUtil::GetInstance()->Dissolve(tmpPolygons);
    }
    else
    {
        neighborAndIntersectionPolygon = tmpPolygons;
    }

    ///
    /// 従道路接続
    ///

    // すでに登録されている主道路の情報を保存
    int mainIntersectionRoadCount = intersectionPolylines.size();

    // 隣接道路の情報
    // 複数車線数に対応
    std::vector<NeighborRoadInfo*> tmpNeighborRoadInfoList;
    for (auto& info : neighborRoadInfoList)
    {
        if (info.RoadCount > 1)
        {
            for (auto& childInfo : info.ChildNeighborRoadInfo)
            {
                tmpNeighborRoadInfoList.emplace_back(&childInfo);
            }
        }
        else
        {
            tmpNeighborRoadInfoList.emplace_back(&info);
        }
    }

    // 道路未接続数
    // 従道路接続処理続行の判断に使用
    int notCreatedLineCount = neighborRoadInfoList.size() - 2;
    int prevNotCreatedLineCount = neighborRoadInfoList.size() - 2;

    // すべての隣接道路が接続できるか
    // これ以上隣接道路が接続できなくなるまで
    do
    {
        prevNotCreatedLineCount = notCreatedLineCount;
        notCreatedLineCount = 0;

        // 隣接道路一つずつ処理
        for (auto neighborRoadInfo : tmpNeighborRoadInfoList)
        {
            // 既に接続済みであればスキップ
            if (neighborRoadInfo->IsConnectedIntersection == true)
            {
                continue;
            }

            Boost3DPointHash neighborPoint = neighborRoadInfo->ConnectPoint;
            CVector2D neighborLineVec = neighborRoadInfo->ConnectVector2D;
            CVector2D neighborPointCVector = CVector2D(neighborPoint.x(), neighborPoint.y());

            ///
            /// 隣接道路の中心線からまっすぐ伸ばす
            ///
            if (neighborRoadInfo->IsConnectedIntersection == false)
            {
                // 作成済み中心線と交差した点の中で一番近いものを保存
                // 交差したポリラインのポインタ、ポリラインのインデックス（i ～ i+1 の間で交差）、交差した点までの距離、交点
                std::tuple<Boost3DHashPolyline*, size_t, double, Boost3DPointHash> closestCrossPoint = std::make_tuple(nullptr, 0, 0.0, Boost3DPointHash());

                for (auto& intersectionPolyline : intersectionPolylines)
                {
                    if (intersectionPolyline.size() < 2)
                    {
                        continue;
                    }

                    for (int i = 0; i < intersectionPolyline.size() - 1; i++)
                    {
                        CVector2D segmentPoint = CVector2D(intersectionPolyline[i].x(), intersectionPolyline[i].y());
                        CVector2D segmentVec = CVector2D(intersectionPolyline[i + 1].x() - intersectionPolyline[i].x(), intersectionPolyline[i + 1].y() - intersectionPolyline[i].y());
                        CVector2D crossPoint;

                        bool isOnLine1 = false;
                        bool isOnLine2 = false;
                        double t = 0.0;
                        double s = 0.0;

                        // セグメント上に交点がある場合
                        if (CGeoUtil::GetCrossPos(neighborLineVec,
                            neighborPointCVector,
                            segmentVec,
                            segmentPoint,
                            crossPoint,
                            isOnLine1,
                            isOnLine2,
                            t,
                            s)
                            && isOnLine2
                            && s >= 0.0
                            && s <= 1.0)
                        {
                            Boost3DPointHash insertPoint = Boost3DPointHash(crossPoint.x, crossPoint.y, (intersectionPolyline[i].z() + intersectionPolyline[i + 1].z()) / 2);

                            // 新たに作成する中心線の交差点ポリゴン内外判定
                            Boost3DHashPolyline insetPolyline;
                            insetPolyline.emplace_back(neighborPoint);
                            insetPolyline.emplace_back(insertPoint);

                            if(CoveredByPolygonsIgnoreZ(insertPoint, intersectionPolygonList)
                                && CheckPolylineProtrudeFromPolygon(insetPolyline, neighborAndIntersectionPolygon) == false)
                            {
                                if (std::get<0>(closestCrossPoint) == nullptr
                                    || std::get<2>(closestCrossPoint) > t)
                                {
                                    closestCrossPoint = std::make_tuple(&intersectionPolyline, i, t, insertPoint);
                                }
                            }
                        }
                    }
                }

                // 交差する点が一つでも見つかったら
                if (std::get<0>(closestCrossPoint) != nullptr)
                {
                    // ぶつかったポリラインの頂点追加
                    Boost3DHashPolyline* crossedPolyline = std::get<0>(closestCrossPoint);
                    size_t crossedPolylineIdx = std::get<1>(closestCrossPoint);
                    Boost3DPointHash crossedPoint = std::get<3>(closestCrossPoint);

                    // 同じ点がある場合は追加しない
                    if ((crossedPolyline->cbegin() + crossedPolylineIdx)->IsRoundEqual(crossedPoint))
                    {
                        crossedPoint = *(crossedPolyline->cbegin() + crossedPolylineIdx);
                    }
                    else if ((crossedPolyline->cbegin() + crossedPolylineIdx + 1)->IsRoundEqual(crossedPoint))
                    {
                        crossedPoint = *(crossedPolyline->cbegin() + crossedPolylineIdx + 1);
                    }
                    else
                    {
                        crossedPolyline->insert(crossedPolyline->cbegin() + crossedPolylineIdx + 1, crossedPoint);
                    }

                    // 新たに作られるポリラインの作成
                    Boost3DHashPolyline creatingPolyline;
                    creatingPolyline.emplace_back(neighborPoint);
                    creatingPolyline.emplace_back(crossedPoint);
                    intersectionPolylines.emplace_back(creatingPolyline);

                    neighborRoadInfo->IsConnectedIntersection = true;
                }
            }

            ///
            /// 作成済みの交差点ポリラインに垂線を引く
            ///
            if (neighborRoadInfo->IsConnectedIntersection == false)
            {
                // 作成済み中心線と交差した点の中で一番近いものを保存
                // 交差したポリラインのポインタ、ポリラインのインデックス（i ～ i+1 の間で交差）、交差した点までの距離、交点
                std::tuple<Boost3DHashPolyline*, size_t, double, Boost3DPointHash> closestCrossPoint = std::make_tuple(nullptr, 0, 0.0, Boost3DPointHash());

                for (auto& intersectionPolyline : intersectionPolylines)
                {
                    if (intersectionPolyline.size() < 2)
                    {
                        continue;
                    }

                    for (int i = 0; i < intersectionPolyline.size() - 1; i++)
                    {
                        CVector2D segmentPoint = CVector2D(intersectionPolyline[i].x(), intersectionPolyline[i].y());
                        CVector2D segmentVec = CVector2D(intersectionPolyline[i + 1].x() - intersectionPolyline[i].x(), intersectionPolyline[i + 1].y() - intersectionPolyline[i].y());
                        CVector2D crossPoint;

                        bool isOnLine = false;
                        double t = 0.0;
                        double dist = 0.0;

                        // セグメント上に交点がある場合
                        if (CGeoUtil::GetCrossPos(neighborPointCVector,
                            segmentVec,
                            segmentPoint,
                            crossPoint,
                            isOnLine,
                            t,
                            dist)
                            && isOnLine
                            && t >= 0.0
                            && t <= 1.0)
                        {
                            Boost3DPointHash insertPoint = Boost3DPointHash(crossPoint.x, crossPoint.y, (intersectionPolyline[i].z() + intersectionPolyline[i + 1].z()) / 2);

                            // 新たに作成する中心線の交差点ポリゴン内外判定
                            Boost3DHashPolyline insetPolyline;
                            insetPolyline.emplace_back(neighborPoint);
                            insetPolyline.emplace_back(insertPoint);

                            if (CoveredByPolygonsIgnoreZ(insertPoint, intersectionPolygonList)
                                && CheckPolylineProtrudeFromPolygon(insetPolyline, neighborAndIntersectionPolygon) == false)
                            {
                                if (std::get<0>(closestCrossPoint) == nullptr
                                    || std::get<2>(closestCrossPoint) > t)
                                {
                                    //closestCrossPoint = std::make_tuple(neighborRoadInfo->CenterLinePtr.get(), i, t, insertPoint);
                                    closestCrossPoint = std::make_tuple(&intersectionPolyline, i, t, insertPoint);
                                }
                            }
                        }
                    }
                }

                // 交差する点が一つでも見つかったら
                if (std::get<0>(closestCrossPoint) != nullptr)
                {
                    // ぶつかったポリラインの頂点追加
                    Boost3DHashPolyline* crossedPolyline = std::get<0>(closestCrossPoint);
                    size_t crossedPolylineIdx = std::get<1>(closestCrossPoint);
                    Boost3DPointHash crossedPoint = std::get<3>(closestCrossPoint);

                    // 同じ点がある場合は追加しない
                    if ((crossedPolyline->cbegin() + crossedPolylineIdx)->IsRoundEqual(crossedPoint))
                    {
                        crossedPoint = *(crossedPolyline->cbegin() + crossedPolylineIdx);
                    }
                    else if ((crossedPolyline->cbegin() + crossedPolylineIdx + 1)->IsRoundEqual(crossedPoint))
                    {
                        crossedPoint = *(crossedPolyline->cbegin() + crossedPolylineIdx + 1);
                    }
                    else
                    {
                        crossedPolyline->insert(crossedPolyline->cbegin() + crossedPolylineIdx + 1, crossedPoint);
                    }

                    // 新たに作られるポリラインの作成
                    Boost3DHashPolyline creatingPolyline;
                    creatingPolyline.emplace_back(neighborPoint);
                    creatingPolyline.emplace_back(crossedPoint);
                    intersectionPolylines.emplace_back(creatingPolyline);

                    neighborRoadInfo->IsConnectedIntersection = true;
                }
            }

            ///
            /// 重心に向かって伸ばす
            ///
            if (neighborRoadInfo->IsConnectedIntersection == false)
            {
                // 作成済み中心線と交差した点の中で一番近いものを保存
                // 交差したポリラインのポインタ、ポリラインのインデックス（i ～ i+1 の間で交差）、交差した点までの距離、交点
                std::tuple<Boost3DHashPolyline*, size_t, double, Boost3DPointHash> closestCrossPoint = std::make_tuple(nullptr, 0, 0.0, Boost3DPointHash());

                // 重心を算出
                Boost3DPointHash centroidPoint;
                bg::centroid(intersectionPolygonList, centroidPoint);
                CVector2D neighbor2middleVec = CVector2D(centroidPoint.x(), centroidPoint.y()) - neighborPointCVector;
                neighbor2middleVec.Normalize();

                for (auto& intersectionPolyline : intersectionPolylines)
                {
                    if (intersectionPolyline.size() < 2)
                    {
                        continue;
                    }

                    for (int i = 0; i < intersectionPolyline.size() - 1; i++)
                    {
                        CVector2D segmentPoint = CVector2D(intersectionPolyline[i].x(), intersectionPolyline[i].y());
                        CVector2D segmentVec = CVector2D(intersectionPolyline[i + 1].x() - intersectionPolyline[i].x(), intersectionPolyline[i + 1].y() - intersectionPolyline[i].y());
                        CVector2D crossPoint;

                        bool isOnLine1 = false;
                        bool isOnLine2 = false;
                        double t = 0.0;
                        double s = 0.0;

                        // セグメント上に交点がある場合
                        if (CGeoUtil::GetCrossPos(neighbor2middleVec,
                            neighborPointCVector,
                            segmentVec,
                            segmentPoint,
                            crossPoint,
                            isOnLine1,
                            isOnLine2,
                            t,
                            s)
                            && isOnLine2
                            && s >= 0.0
                            && s <= 1.0)
                        {
                            Boost3DPointHash insertPoint = Boost3DPointHash(crossPoint.x, crossPoint.y, (intersectionPolyline[i].z() + intersectionPolyline[i + 1].z()) / 2);

                            // 新たに作成する中心線の交差点ポリゴン内外判定
                            Boost3DHashPolyline insetPolyline;
                            insetPolyline.emplace_back(neighborPoint);
                            insetPolyline.emplace_back(insertPoint);

                            if (CoveredByPolygonsIgnoreZ(insertPoint, intersectionPolygonList)
                                && CheckPolylineProtrudeFromPolygon(insetPolyline, neighborAndIntersectionPolygon) == false) // TODO 重心は最終手段のため、ポリゴン内判定を削除するか検討する
                            {
                                if (std::get<0>(closestCrossPoint) == nullptr
                                    || std::get<2>(closestCrossPoint) > t)
                                {
                                    closestCrossPoint = std::make_tuple(&intersectionPolyline, i, t, insertPoint);
                                }
                            }
                        }
                    }
                }

                // 交差する点が一つでも見つかったら
                if (std::get<0>(closestCrossPoint) != nullptr)
                {
                    // ぶつかったポリラインの頂点追加
                    Boost3DHashPolyline* crossedPolyline = std::get<0>(closestCrossPoint);
                    size_t crossedPolylineIdx = std::get<1>(closestCrossPoint);
                    Boost3DPointHash crossedPoint = std::get<3>(closestCrossPoint);

                    // 同じ点がある場合は追加しない
                    if ((crossedPolyline->cbegin() + crossedPolylineIdx)->IsRoundEqual(crossedPoint))
                    {
                        crossedPoint = *(crossedPolyline->cbegin() + crossedPolylineIdx);
                    }
                    else if ((crossedPolyline->cbegin() + crossedPolylineIdx + 1)->IsRoundEqual(crossedPoint))
                    {
                        crossedPoint = *(crossedPolyline->cbegin() + crossedPolylineIdx + 1);
                    }
                    else
                    {
                        crossedPolyline->insert(crossedPolyline->cbegin() + crossedPolylineIdx + 1, crossedPoint);
                    }

                    // 新たに作られるポリラインの作成
                    Boost3DHashPolyline creatingPolyline;
                    creatingPolyline.emplace_back(neighborPoint);
                    creatingPolyline.emplace_back(crossedPoint);
                    intersectionPolylines.emplace_back(creatingPolyline);

                    neighborRoadInfo->IsConnectedIntersection = true;
                }
            }

            ///
            /// 交点が見つかった場合
            ///
            if (neighborRoadInfo->IsConnectedIntersection == false)
            {
                // 交点が見つからなかった場合
                notCreatedLineCount++;
            }
        }
    } while (notCreatedLineCount != 0
        && notCreatedLineCount < prevNotCreatedLineCount);

    ///
    /// 不要な頂点を削除する
    ///
    for (auto& intersectionPolyline : intersectionPolylines)
    {
        if (intersectionPolyline.size() < 3)
        {
            continue;
        }

        // 始点と終点は削除しない
        for (int i = 1; i < intersectionPolyline.size() - 1; i++)
        {
            Boost3DPointHash targetPoint = intersectionPolyline[i];
            CVector2D target2PrevVec2d = CVector2D(intersectionPolyline[i - 1].x(), intersectionPolyline[i - 1].y()) - CVector2D(targetPoint.x(), targetPoint.y());
            CVector2D target2NextVec2d = CVector2D(intersectionPolyline[i + 1].x(), intersectionPolyline[i + 1].y()) - CVector2D(targetPoint.x(), targetPoint.y());

            // この交点が他のポリラインと接しているか
            bool isOtherCross = false;

            // 他に対象の点を含むポリラインがないか検索
            for (auto& tmpIntersectionPolyline : intersectionPolylines)
            {
                // 同じポリラインについては確認しない
                //if (bg::equals(tmpIntersectionPolyline, intersectionPolyline))
                if (&tmpIntersectionPolyline == &intersectionPolyline)
                {
                    continue;
                }

                // ポリライン内のすべての点を検索
                for (auto& tmpPoint : tmpIntersectionPolyline)
                {
                    if (tmpPoint.IsRoundEqual(targetPoint))
                    {
                        isOtherCross = true;
                        break;
                    }
                }

                if (isOtherCross)
                {
                    break;
                }
            }

            // 他のポリラインと接しておらず
            // また、前後のベクトルがほぼ平行の場合
            if (isOtherCross == false
                && IsParallel(target2PrevVec2d, target2NextVec2d, -1))
            {
                // 対象の点を削除する
                intersectionPolyline.erase((intersectionPolyline.cbegin() + i));
                i--;
            }
        }
    }

    ///
    /// LOD1以外
    /// かつ、主道路が2以上
    /// かつ、サブ道路が1本以上作成できた場合は
    /// 主道路間の中心線を作成する
    ///
    if (m_iLod != 1
        && mainIntersectionRoadCount > 1
        && intersectionPolylines.size() > mainIntersectionRoadCount)
    {

        size_t mainIntersectionRoadEndIndex = mainIntersectionRoadCount - 1;
        size_t subIntersectionRoadStartindex = mainIntersectionRoadCount;
        std::vector<std::pair<Boost3DHashPolyline*, int>> mainCenterLineCrossedSubLineCountList; // メイン道路ポインタ、交差数

        // 各メインポリラインがサブポリラインと交差している数を調べる
        for (int i = 0; i <= mainIntersectionRoadEndIndex; i++)
        {
            mainCenterLineCrossedSubLineCountList.emplace_back(std::make_pair(&(intersectionPolylines[i]), intersectionPolylines[i].size() - 2));
        }

        // 交差数が多い順にソート
        std::sort(mainCenterLineCrossedSubLineCountList.begin(), mainCenterLineCrossedSubLineCountList.end(),
            [](std::pair<Boost3DHashPolyline*, int>& data1, std::pair<Boost3DHashPolyline*, int>& data2)
            {
                return data1.second > data2.second;
            });

        // サブ道路の交点を持つメイン道路が3つ以上ある場合
        // 不正な接続とする
        if (mainCenterLineCrossedSubLineCountList.size() > 2
            && mainCenterLineCrossedSubLineCountList[2].second > 0)
        {
            // 不正な接続のため、特に処理を行わない
        }
        // 間に埋めるポリラインを求める
        else if (mainCenterLineCrossedSubLineCountList.size() > 1)
        {
            Boost3DHashMultiLines additionalCenterLineList;

            // 交差数がすべて同数の場合
            // 交差しないようにつなぎ合わせる
            if (mainCenterLineCrossedSubLineCountList[0].second == mainCenterLineCrossedSubLineCountList[1].second)
            {
                Boost3DMultiPointHashs mainCrossPointList1;
                Boost3DMultiPointHashs mainCrossPointList2;
                std::vector<std::pair<size_t, size_t>> connectPairList;

                for (int j = 1; j < mainCenterLineCrossedSubLineCountList[0].first->size() - 1; j++)
                {
                    mainCrossPointList1.emplace_back((*mainCenterLineCrossedSubLineCountList[0].first)[j]);
                }
                for (int j = 1; j < mainCenterLineCrossedSubLineCountList[1].first->size() - 1; j++)
                {
                    mainCrossPointList2.emplace_back((*mainCenterLineCrossedSubLineCountList[1].first)[j]);
                }

                // 交点同士のペアを求める
                if (PairMultiplePoints(mainCrossPointList1, mainCrossPointList2, connectPairList))
                {
                    for (auto connectPointPair : connectPairList)
                    {
                        Boost3DHashPolyline additionalCenterLine;
                        additionalCenterLine.emplace_back((*mainCenterLineCrossedSubLineCountList[0].first)[connectPointPair.first + 1]);
                        additionalCenterLine.emplace_back((*mainCenterLineCrossedSubLineCountList[1].first)[connectPointPair.second + 1]);

                        // 島ポリゴンと重ならなければ追加
                        bool isCrossIsland = false;
                        for (auto islandPolygon : intersectionIslandOnlyPolygonList)
                        {
                            if(CBoostGeoUtil::Disjoint(islandPolygon,additionalCenterLine) == false)
                            {
                                isCrossIsland = true;
                                break;
                            }
                        }
                        if (isCrossIsland == false)
                        {
                            additionalCenterLineList.emplace_back(additionalCenterLine);
                        }
                    }
                }
            }
            // 交差数が異なる場合で、一つが交差数0の場合
            // 各交点から垂直にもう一つの辺まで伸ばす
            else if (mainCenterLineCrossedSubLineCountList[1].second == 0)
            {
                Boost3DHashPolyline* multiCrossCenterLinePtr = mainCenterLineCrossedSubLineCountList[0].first;
                Boost3DHashPolyline* notCrossCenterLinePtr = mainCenterLineCrossedSubLineCountList[1].first;

                // 各交点を求める
                for (int i = 1; i < multiCrossCenterLinePtr->size() - 1; i++)
                {
                    auto firstPoint = (*multiCrossCenterLinePtr)[i];
                    auto secondFrontPoint = (*notCrossCenterLinePtr)[0];
                    auto secondBackPoint = (*notCrossCenterLinePtr)[1];

                    CVector2D firstPosVec2d = CVector2D(firstPoint.x(), firstPoint.y());
                    CVector2D secondVecVec2d = CVector2D(secondBackPoint.x() - secondFrontPoint.x(), secondBackPoint.y() - secondFrontPoint.y());
                    CVector2D secondPosVec2d = CVector2D(secondFrontPoint.x(), secondFrontPoint.y());
                    CVector2D crossPosVec2d;
                    bool isOnLine = false;
                    double t = 0.0;
                    double dist = 0.0;

                    // 頂点からメイン道路に向かって垂線を引く
                    if (CGeoUtil::GetCrossPos(firstPosVec2d, secondVecVec2d, secondPosVec2d, crossPosVec2d, isOnLine, t, dist)
                        && isOnLine == true
                        && CEpsUtil::Greater(t, 0.0)
                        && CEpsUtil::LessEqual(t, 1.0))
                    {
                        Boost3DPointHash crossPoint = Boost3DPointHash(
                            secondFrontPoint.x() + (secondBackPoint.x() - secondFrontPoint.x()) * t,
                            secondFrontPoint.y() + (secondBackPoint.y() - secondFrontPoint.y()) * t,
                            secondFrontPoint.z() + (secondBackPoint.z() - secondFrontPoint.z()) * t
                        );

                        Boost3DHashPolyline additionalCenterLine;
                        additionalCenterLine.emplace_back(firstPoint);
                        additionalCenterLine.emplace_back(crossPoint);

                        // 島ポリゴンと重ならなければ追加
                        bool isCrossIsland = false;
                        for (auto islandPolygon : intersectionIslandOnlyPolygonList)
                        {
                            if (CBoostGeoUtil::Disjoint(islandPolygon, additionalCenterLine) == false)
                            {
                                isCrossIsland = true;
                                break;
                            }
                        }
                        if (isCrossIsland == false)
                        {
                            additionalCenterLineList.emplace_back(additionalCenterLine);
                        }
                    }
                }
            }
            // 交差数が異なる場合で、一つが交差数1の場合
            // 残りの交点と接続する
            else if (mainCenterLineCrossedSubLineCountList[0].second == 1
                || mainCenterLineCrossedSubLineCountList[1].second == 1)
            {
                Boost3DHashPolyline* multiCrossCenterLinePtr;
                Boost3DPointHash singleCrossPoint;

                if (mainCenterLineCrossedSubLineCountList[0].second == 1)
                {
                    singleCrossPoint = (*mainCenterLineCrossedSubLineCountList[0].first)[1];
                    multiCrossCenterLinePtr = mainCenterLineCrossedSubLineCountList[1].first;
                }
                else
                {
                    multiCrossCenterLinePtr = mainCenterLineCrossedSubLineCountList[0].first;
                    singleCrossPoint = (*mainCenterLineCrossedSubLineCountList[1].first)[1];
                }

                for (int i = 1; i < multiCrossCenterLinePtr->size() - 1; i++)
                {
                    Boost3DHashPolyline additionalCenterLine;
                    additionalCenterLine.emplace_back(singleCrossPoint);
                    additionalCenterLine.emplace_back((*multiCrossCenterLinePtr)[i]);

                    // 島ポリゴンと重ならなければ追加
                    bool isCrossIsland = false;
                    for (auto islandPolygon : intersectionIslandOnlyPolygonList)
                    {
                        if (CBoostGeoUtil::Disjoint(islandPolygon, additionalCenterLine) == false)
                        {
                            isCrossIsland = true;
                            break;
                        }
                    }
                    if (isCrossIsland == false)
                    {
                        additionalCenterLineList.emplace_back(additionalCenterLine);
                    }
                }
            }
            // それ以外の場合
            // 各サブ道路の接続点を一つにまとめてつなぎ合わせる
            else
            {
                Boost3DHashPolyline* firstMultiCrossCenterLinePtr = mainCenterLineCrossedSubLineCountList[0].first;
                Boost3DHashPolyline* secondMultiCrossCenterLinePtr = mainCenterLineCrossedSubLineCountList[1].first;

                // サブ道路の修正用にメイン道路から削除した頂点座標を保存する
                std::vector<Boost3DPointHash> firstErasedMainCenterLineCrossPointList;
                std::vector<Boost3DPointHash> secondErasedMainCenterLineCrossPointList;

                // 各交点の中心を求める
                // 交点数が偶数の場合は中心2点の中点
                // 交点数が奇数の場合は中心の点
                Boost3DPointHash firstCrossPoint;
                Boost3DPointHash secondCrossPoint;
                if (firstMultiCrossCenterLinePtr->size() % 2 == 0)
                {
                    int crossPointBackIndex = firstMultiCrossCenterLinePtr->size() / 2;
                    int crossPointFrontIndex = crossPointBackIndex - 1;
                    firstCrossPoint = Boost3DPointHash(
                        ((*firstMultiCrossCenterLinePtr)[crossPointFrontIndex].x() + (*firstMultiCrossCenterLinePtr)[crossPointBackIndex].x()) / 2,
                        ((*firstMultiCrossCenterLinePtr)[crossPointFrontIndex].y() + (*firstMultiCrossCenterLinePtr)[crossPointBackIndex].y()) / 2,
                        ((*firstMultiCrossCenterLinePtr)[crossPointFrontIndex].z() + (*firstMultiCrossCenterLinePtr)[crossPointBackIndex].z()) / 2
                    );

                    // メイン道路の頂点を削除、新しい中点は追加
                    for (int i = 1; i < firstMultiCrossCenterLinePtr->size() - 1; i++)
                    {
                        // 削除する頂点を保存する
                        firstErasedMainCenterLineCrossPointList.emplace_back(*(firstMultiCrossCenterLinePtr->cbegin() + i));

                        // 頂点削除
                        firstMultiCrossCenterLinePtr->erase(firstMultiCrossCenterLinePtr->cbegin() + i);

                        // 頂点追加
                        if (i == crossPointFrontIndex)
                        {
                            firstMultiCrossCenterLinePtr->insert(firstMultiCrossCenterLinePtr->cbegin() + i, firstCrossPoint);
                        }
                    }
                }
                else
                {
                    int crossPointIndex = firstMultiCrossCenterLinePtr->size() / 2;
                    firstCrossPoint = (*firstMultiCrossCenterLinePtr)[crossPointIndex];

                    // メイン道路の頂点を削除
                    for (int i = 1; i < firstMultiCrossCenterLinePtr->size() - 1; i++)
                    {
                        // 中点は削除しない
                        if (i == crossPointIndex)
                        {
                            continue;
                        }

                        // 削除する頂点を保存する
                        firstErasedMainCenterLineCrossPointList.emplace_back(*(firstMultiCrossCenterLinePtr->cbegin() + i));

                        // 頂点削除
                        firstMultiCrossCenterLinePtr->erase(firstMultiCrossCenterLinePtr->cbegin() + i);
                    }
                }
                if (secondMultiCrossCenterLinePtr->size() % 2 == 0)
                {
                    int crossPointBackIndex = secondMultiCrossCenterLinePtr->size() / 2;
                    int crossPointFrontIndex = crossPointBackIndex - 1;
                    secondCrossPoint = Boost3DPointHash(
                        ((*secondMultiCrossCenterLinePtr)[crossPointFrontIndex].x() + (*secondMultiCrossCenterLinePtr)[crossPointBackIndex].x()) / 2,
                        ((*secondMultiCrossCenterLinePtr)[crossPointFrontIndex].y() + (*secondMultiCrossCenterLinePtr)[crossPointBackIndex].y()) / 2,
                        ((*secondMultiCrossCenterLinePtr)[crossPointFrontIndex].z() + (*secondMultiCrossCenterLinePtr)[crossPointBackIndex].z()) / 2
                    );

                    // メイン道路の頂点を削除、新しい中点は追加
                    for (int i = 1; i < secondMultiCrossCenterLinePtr->size() - 1; i++)
                    {
                        // 削除する頂点を保存する
                        secondErasedMainCenterLineCrossPointList.emplace_back(*(secondMultiCrossCenterLinePtr->cbegin() + i));

                        // 頂点削除
                        secondMultiCrossCenterLinePtr->erase(secondMultiCrossCenterLinePtr->cbegin() + i);

                        // 頂点追加
                        if (i == crossPointFrontIndex)
                        {
                            secondMultiCrossCenterLinePtr->insert(secondMultiCrossCenterLinePtr->cbegin() + i, firstCrossPoint);
                        }
                    }
                }
                else
                {
                    int crossPointIndex = secondMultiCrossCenterLinePtr->size() / 2;
                    secondCrossPoint = (*secondMultiCrossCenterLinePtr)[crossPointIndex];

                    // メイン道路の頂点を削除
                    for (int i = 1; i < secondMultiCrossCenterLinePtr->size() - 1; i++)
                    {
                        // 中点は削除しない
                        if (i == crossPointIndex)
                        {
                            continue;
                        }

                        // 削除する頂点を保存する
                        secondErasedMainCenterLineCrossPointList.emplace_back(*(secondMultiCrossCenterLinePtr->cbegin() + i));

                        // 頂点削除
                        secondMultiCrossCenterLinePtr->erase(secondMultiCrossCenterLinePtr->cbegin() + i);
                    }
                }

                // サブ道路の交点を修正
                for (int i = subIntersectionRoadStartindex; i < intersectionPolylines.size(); i++)
                {
                    bool isUpdate = false;

                    for (auto firstErasePoint : firstErasedMainCenterLineCrossPointList)
                    {
                        if (firstErasePoint.IsRoundEqual(intersectionPolylines[i].back()))
                        {
                            intersectionPolylines[i].erase(intersectionPolylines[i].cend() - 1);
                            intersectionPolylines[i].insert(intersectionPolylines[i].cend(), firstCrossPoint);

                            isUpdate = true;
                        }
                    }

                    if (isUpdate)
                    {
                        continue;
                    }

                    for (auto secondErasePoint : secondErasedMainCenterLineCrossPointList)
                    {
                        if (secondErasePoint.IsRoundEqual(intersectionPolylines[i].back()))
                        {
                            intersectionPolylines[i].erase(intersectionPolylines[i].cend() - 1);
                            intersectionPolylines[i].insert(intersectionPolylines[i].cend(), secondCrossPoint);

                            isUpdate = true;
                        }
                    }
                }

                // 追加するポリラインを追加する
                Boost3DHashPolyline additionalCenterLine;
                additionalCenterLine.emplace_back(firstCrossPoint);
                additionalCenterLine.emplace_back(secondCrossPoint);

                // 島ポリゴンと重ならなければ追加
                bool isCrossIsland = false;
                for (auto islandPolygon : intersectionIslandOnlyPolygonList)
                {
                    if (CBoostGeoUtil::Disjoint(islandPolygon, additionalCenterLine) == false)
                    {
                        isCrossIsland = true;
                        break;
                    }
                }
                if (isCrossIsland == false)
                {
                    additionalCenterLineList.emplace_back(additionalCenterLine);
                }

            }

            // 作成したポリラインが他のメイン道路に交わっていないか確認する
            for (auto& additionalPolyline : additionalCenterLineList)
            {
                std::vector<std::pair<Boost3DPointHash, double>> crossPointAndDistanceFromStartPairList; // 交点、additionalPolylineの始点からの距離

                // メイン道路のうち交点が発生しない内側の線を総検索、
                for (int i = 2; i < mainCenterLineCrossedSubLineCountList.size(); i++)
                {
                    // ポリラインの交点と
                    // 作成したポリラインの始点からの距離を求める
                    Boost3DPointHash additionalStartPoint = additionalPolyline[0];
                    Boost3DPointHash additionalBackPoint = additionalPolyline[1];

                    CVector2D additionalStartPointVec2d = CVector2D(additionalStartPoint.x(), additionalStartPoint.y());
                    CVector2D additionalBackPointVec2d = CVector2D(additionalBackPoint.x(), additionalBackPoint.y());
                    CVector2D additionalVectorVec2d = additionalBackPointVec2d - additionalStartPointVec2d;

                    for (int j = 0; j < mainCenterLineCrossedSubLineCountList[i].first->size() - 1; j++)
                    {
                        Boost3DPointHash searchStartPoint = (*mainCenterLineCrossedSubLineCountList[i].first)[j];
                        Boost3DPointHash searchBackPoint = (*mainCenterLineCrossedSubLineCountList[i].first)[j + 1];

                        CVector2D searchStartPointVec2d = CVector2D(searchStartPoint.x(), searchStartPoint.y());
                        CVector2D searchBackPointVec2d = CVector2D(searchBackPoint.x(), searchBackPoint.y());
                        CVector2D searchBackVectorVec2d = searchBackPointVec2d - searchStartPointVec2d;
                        CVector2D crossPointVec2d;

                        bool isOnLine1 = false;
                        bool isOnLine2 = false;
                        double t = 0.0;
                        double s = 0.0;

                        if (CGeoUtil::GetCrossPos(additionalVectorVec2d, additionalStartPointVec2d, searchBackVectorVec2d, searchStartPointVec2d, crossPointVec2d, isOnLine1, isOnLine2, t, s)
                            && isOnLine1
                            && isOnLine2
                            && CEpsUtil::Greater(t, 0.0)
                            && CEpsUtil::LessEqual(t, 1.0)
                            && CEpsUtil::Greater(s, 0.0)
                            && CEpsUtil::LessEqual(s, 1.0))
                        {
                            Boost3DPointHash crossPoint = Boost3DPointHash(
                                additionalStartPoint.x() + (additionalBackPoint.x() - additionalStartPoint.x()) * t,
                                additionalStartPoint.y() + (additionalBackPoint.y() - additionalStartPoint.y()) * t,
                                additionalStartPoint.z() + (additionalBackPoint.z() - additionalStartPoint.z()) * t
                            );

                            crossPointAndDistanceFromStartPairList.emplace_back(std::make_pair(crossPoint, bg::distance(additionalStartPoint, crossPoint)));

                            break;
                        }
                    }
                }

                // additionalPolylineの始点から近い順にソート
                std::sort(
                    crossPointAndDistanceFromStartPairList.begin(),
                    crossPointAndDistanceFromStartPairList.end(),
                    [](std::pair<Boost3DPointHash, double>& data1, std::pair<Boost3DPointHash, double>& data2)
                    {
                        return CEpsUtil::Less(data1.second, data2.second);
                    });

                // additionalPolylineに頂点追加
                for (auto& crossPair : crossPointAndDistanceFromStartPairList)
                {
                    additionalPolyline.insert(additionalPolyline.cend() - 1, crossPair.first);
                }
            }

            // 作成したポリラインを基に対象のサブ道路を伸ばす
            for (int i = subIntersectionRoadStartindex; i < intersectionPolylines.size(); i++)
            {
                // 伸ばす対象のサブ道路
                Boost3DHashPolyline* subPolylinePtr = &(intersectionPolylines[i]);

                // 追加するポリラインを総検索
                for(int j = 0; j < additionalCenterLineList.size(); j++)
                {
                    Boost3DHashPolyline& searchAdditionalPolyline = additionalCenterLineList[j];

                    // サブ道路の終点と追加ポリラインの始点が一致すれば
                    // 追加ポリラインの頂点（始点を除く）をサブ道路に追加する
                    if (subPolylinePtr->back().IsRoundEqual(searchAdditionalPolyline.front()))
                    {
                        for (int k = 1; k < searchAdditionalPolyline.size(); k++)
                        {
                            subPolylinePtr->insert(subPolylinePtr->cend(), searchAdditionalPolyline[k]);
                        }

                        // 追加したポリラインは重複追加しないように削除する
                        additionalCenterLineList.erase(additionalCenterLineList.cbegin() + j);

                        break;
                    }
                    // サブ道路の終点と追加ポリラインの終点が一致すれば
                    // 追加ポリラインの頂点（終点を除く）を後ろからサブ道路に追加する
                    else if (subPolylinePtr->back().IsRoundEqual(searchAdditionalPolyline.back()))
                    {
                        for (int k = 1; k < searchAdditionalPolyline.size(); k++)
                        {
                            subPolylinePtr->insert(subPolylinePtr->cend(), searchAdditionalPolyline[searchAdditionalPolyline.size() - 1 - k]);
                        }

                        // 追加したポリラインは重複追加しないように削除する
                        additionalCenterLineList.erase(additionalCenterLineList.cbegin() + j);

                        break;
                    }
                }
            }
        }
    }

    ///
    /// 主道路を分割して
    /// すべての交差点中心線が交差点の中心に向かうようにする
    ///
    for (int i = 0; i < mainIntersectionRoadCount; i++)
    {
        Boost3DHashPolyline& currentMainCenterLine = intersectionPolylines[i];

        // 途中に交点がなければスキップ
        if (currentMainCenterLine.size() < 3)
        {
            continue;
        }

        Boost3DHashPolyline firstMainCenterLine;
        Boost3DHashPolyline secondMainCenterLine;

        for (int j = 0; j < currentMainCenterLine.size(); j++)
        {
            // 交差点内の中心線は(隣接道路)→(交差点の中心点)とする
            if (j < 2)
            {
                // 前から追加する
                firstMainCenterLine.emplace_back(currentMainCenterLine[j]);
            }

            if(j > 0)
            {
                // 後ろから追加する
                secondMainCenterLine.emplace(secondMainCenterLine.cbegin(), currentMainCenterLine[j]);
            }
        }

        // 主道路の新しく置き換える
        intersectionPolylines.erase(intersectionPolylines.cbegin() + i);
        intersectionPolylines.insert(intersectionPolylines.cbegin() + i, firstMainCenterLine);
        intersectionPolylines.insert(intersectionPolylines.cbegin() + i + 1, secondMainCenterLine);

        // メイン道路が一つ増えた分、値を調整する
        i++;
        mainIntersectionRoadCount++;
    }

    // 親の隣接道路情報を更新する
    for (auto& info : neighborRoadInfoList)
    {
        if (info.RoadCount < 2)
        {
            continue;
        }

        for (auto& childInfo : info.ChildNeighborRoadInfo)
        {
            // 子が一つでも接続が完了していればtrueとする
            if (childInfo.IsConnectedIntersection)
            {
                info.IsConnectedIntersection = true;
                break;
            }
        }
    }

    return true;
}

/*!
 * @brief 異なる中心線数の接続
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::ConnectNeighborRoad()
{
    // LOD1は複数の車道が存在しないため終了
    if (m_iLod == 1)
    {
        return true;
    }

    // すべての道路を検索
    for (auto& tranRoadDataPtr : m_tranRoadData)
    {
        // この道路に交差点を含むか調べる
        bool isIntersection = tranRoadDataPtr->m_nInOut > 2;
        switch (m_iLod)
        {
        case 2:
            for (auto item : tranRoadDataPtr->m_lod2List)
            {
                if (item.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION)
                {
                    isIntersection = true;
                    break;
                }
            }
            break;
        case 3:
            for (auto item : tranRoadDataPtr->m_lod3List)
            {
                if (item.m_fuctionType == (int)TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION)
                {
                    isIntersection = true;
                    break;
                }
            }
            break;
        case 1:
        default:
            break;
        }

        auto& centerLineList = tranRoadDataPtr->roadCenterLineList;

        // 隣接道路が交差部の場合
        // その隣接道路もまとめて交差点接続する
        std::set<std::shared_ptr<CTranRoadData>> neighborRoadPtrList;
        if (isIntersection)
        {
            std::set<std::shared_ptr<CTranRoadData>> intersectionRoadList; // GetNeighborRoadOfIntersection実行用
            if (GetNeighborRoadOfIntersection(tranRoadDataPtr, neighborRoadPtrList, intersectionRoadList) == false)
            {
                continue;
            }
        }
        else
        {
            neighborRoadPtrList = tranRoadDataPtr->m_neighborRoadPtr;
        }

        // すべての中心線を検索
        for (auto& centerLineData : centerLineList)
        {
            Boost3DHashPolyline *centerLinePtr = &(centerLineData->centerLine);
            bool isConnectCenterLineFront = false;
            bool isConnectCenterLineBack = false;

            // 中心線の始点または終点が隣接道路の中心線の始点または終点と一致しているか確認
            for (auto& neighborRoadPtr : neighborRoadPtrList)
            {
                for (auto& neighborCenterLineData : neighborRoadPtr->roadCenterLineList)
                {
                    if (neighborCenterLineData->centerLine.front().IsRoundEqual(centerLinePtr->front())
                        || neighborCenterLineData->centerLine.back().IsRoundEqual(centerLinePtr->front()))
                    {
                        // このcenterLinePtrは中心線がつながっている
                        isConnectCenterLineFront = true;
                    }

                    if (neighborCenterLineData->centerLine.front().IsRoundEqual(centerLinePtr->back())
                        || neighborCenterLineData->centerLine.back().IsRoundEqual(centerLinePtr->back()))
                    {
                        // このcenterLinePtrは中心線がつながっている
                        isConnectCenterLineBack = true;
                    }

                    if (isConnectCenterLineFront && isConnectCenterLineBack)
                    {
                        break;
                    }
                }

                if (isConnectCenterLineFront && isConnectCenterLineBack)
                {
                    break;
                }
            }

            // すでに中心線の始点と終点が他の中心線と接続できていることが確認できれば
            // 次の中心線に進む
            if (isConnectCenterLineFront && isConnectCenterLineBack)
            {
                continue;
            }

            // 中心線の始点または終点が同じ道路の中心線の始点または終点と一致しているか確認
            // 同じ道路の中心線とは中心線の途中で交差している場合も考慮する
            for (auto& anothierCenterLineData : centerLineList)
            {
                if (anothierCenterLineData == centerLineData)
                {
                    continue;
                }

                for (auto& vertex : anothierCenterLineData->centerLine)
                {
                    if (vertex.IsRoundEqual(centerLinePtr->front()))
                    {
                        // このcenterLinePtrは中心線がつながっている
                        isConnectCenterLineFront = true;
                    }

                    if (vertex.IsRoundEqual(centerLinePtr->back()))
                    {
                        // このcenterLinePtrは中心線がつながっている
                        isConnectCenterLineBack = true;
                    }

                    if (isConnectCenterLineFront && isConnectCenterLineBack)
                    {
                        break;
                    }
                }

                if (isConnectCenterLineFront && isConnectCenterLineBack)
                {
                    break;
                }
            }

            // 中心線の始点が隣接道路と接続されていない場合
            if (isConnectCenterLineFront == false)
            {
                std::shared_ptr<CTranRoadData> targetNeighborRoadPtr = nullptr;
                std::shared_ptr<CCenterLineData> targetNeighborCenterLinePtr = nullptr;

                // 一番近い距離を保存
                // 幅員の長さより近い道路を検索する
                double minDistance = centerLineData->dMinWidth;

                // 隣接道路の内、中心線の始点に最も近い道路を取得
                for (auto& neighborRoadPtr : neighborRoadPtrList)
                {
                    for (auto& neighborCenterLineData : neighborRoadPtr->roadCenterLineList)
                    {
                        double tmpFrontDistance = bg::distance(CBoostGeoUtil::Conv(centerLinePtr->front()), CBoostGeoUtil::Conv(neighborCenterLineData->centerLine.front()));
                        double tmpBackDistance = bg::distance(CBoostGeoUtil::Conv(centerLinePtr->front()), CBoostGeoUtil::Conv(neighborCenterLineData->centerLine.back()));


                        if (CEpsUtil::Less(tmpFrontDistance, minDistance))
                        {
                            minDistance = tmpFrontDistance;
                            targetNeighborRoadPtr = neighborRoadPtr;
                            targetNeighborCenterLinePtr = neighborCenterLineData;
                        }
                        else if (CEpsUtil::Less(tmpBackDistance, minDistance))
                        {
                            minDistance = tmpBackDistance;
                            targetNeighborRoadPtr = neighborRoadPtr;
                            targetNeighborCenterLinePtr = neighborCenterLineData;
                        }
                    }
                }

                if (targetNeighborRoadPtr != nullptr)
                {
                    // 中心線の端点の近いほうと接続する
                    if (CEpsUtil::Less(
                        bg::distance(centerLinePtr->front(), targetNeighborCenterLinePtr->centerLine.front()),
                        bg::distance(centerLinePtr->front(), targetNeighborCenterLinePtr->centerLine.back())
                    ))
                    {
                        centerLinePtr->erase(centerLinePtr->begin());
                        centerLinePtr->insert(centerLinePtr->begin(), targetNeighborCenterLinePtr->centerLine.front());
                    }
                    else
                    {
                        centerLinePtr->erase(centerLinePtr->begin());
                        centerLinePtr->insert(centerLinePtr->begin(), targetNeighborCenterLinePtr->centerLine.back());
                    }
                }
            }

            // 中心線の終点が隣接道路と接続されていない場合
            if (isConnectCenterLineBack == false)
            {
                std::shared_ptr<CTranRoadData> targetNeighborRoadPtr = nullptr;
                std::shared_ptr<CCenterLineData> targetNeighborCenterLinePtr = nullptr;

                // 一番近い距離を保存
                // 幅員の長さより近い道路を検索する
                double minDistance = centerLineData->dMinWidth;

                // 隣接道路の内、中心線の終点に最も近い道路を取得
                for (auto& neighborRoadPtr : neighborRoadPtrList)
                {
                    for (auto& neighborCenterLineData : neighborRoadPtr->roadCenterLineList)
                    {
                        double tmpFrontDistance = bg::distance(CBoostGeoUtil::Conv(centerLinePtr->back()), CBoostGeoUtil::Conv(neighborCenterLineData->centerLine.front()));
                        double tmpBackDistance = bg::distance(CBoostGeoUtil::Conv(centerLinePtr->back()), CBoostGeoUtil::Conv(neighborCenterLineData->centerLine.back()));

                        if (CEpsUtil::Less(tmpFrontDistance, minDistance))
                        {
                            minDistance = tmpFrontDistance;
                            targetNeighborRoadPtr = neighborRoadPtr;
                            targetNeighborCenterLinePtr = neighborCenterLineData;
                        }
                        else if (CEpsUtil::Less(tmpBackDistance, minDistance))
                        {
                            minDistance = tmpBackDistance;
                            targetNeighborRoadPtr = neighborRoadPtr;
                            targetNeighborCenterLinePtr = neighborCenterLineData;
                        }
                    }
                }

                if (targetNeighborRoadPtr != nullptr)
                {
                    // 中心線の端点の近いほうと接続する
                    if (CEpsUtil::Less(
                        bg::distance(centerLinePtr->back(), targetNeighborCenterLinePtr->centerLine.front()),
                        bg::distance(centerLinePtr->back(), targetNeighborCenterLinePtr->centerLine.back())
                    ))
                    {
                        centerLinePtr->erase(centerLinePtr->end() - 1);
                        centerLinePtr->insert(centerLinePtr->end(), targetNeighborCenterLinePtr->centerLine.front());
                    }
                    else
                    {
                        centerLinePtr->erase(centerLinePtr->end() - 1);
                        centerLinePtr->insert(centerLinePtr->end(), targetNeighborCenterLinePtr->centerLine.back());
                    }
                }
            }
        }
    }

    return true;
}

/*!
 * @brief 2つのポリゴンが重畳しているかどうか判別する
 * @param targetPolygon                     入力ポリゴン1
 * @param searchPolygon                     入力ポリゴン2
 * @param targetOverlapEdgeIndexPairList    重畳している辺の始点と終点のペアリスト
 * @return 処理結果
 * @retval true     重畳している
 * @retval false    重畳していない
*/
bool CNetworkCreator::IsOverlapPolygons(
    Boost3DHashPolygon& targetPolygon,
    Boost3DHashPolygon& searchPolygon,
    std::vector<std::pair<size_t, size_t>>& targetOverlapEdgeIndexPairList)
{
    if (bg::is_empty(targetPolygon)
        || bg::is_empty(searchPolygon))
    {
        return false;
    }

    bool isOverlapStart = false;
    bool isOverlapEnd = false;
    int startIdx = 0;

    // 各セグメント毎に確認
    // 複数セグメントは考慮しない
    for (int i = 0; i < targetPolygon.outer().size() - 1; i++)
    {
        isOverlapStart = false;

        Boost3DPointHash targetStartPoint = targetPolygon.outer()[i];
        Boost3DPointHash targetEndPoint = targetPolygon.outer()[i + 1];

        for (int j = 0; j < searchPolygon.outer().size() - 1; j++)
        {
            Boost3DPointHash searchStartPoint = searchPolygon.outer()[j];
            Boost3DPointHash searchEndPoint = searchPolygon.outer()[j + 1];

            if ((targetStartPoint.IsRoundEqual(searchStartPoint) && targetEndPoint.IsRoundEqual(searchEndPoint))
                || (targetStartPoint.IsRoundEqual(searchEndPoint) && targetEndPoint.IsRoundEqual(searchStartPoint)))
            {
                isOverlapStart = true;
                break;
            }
        }

        if (isOverlapStart)
        {
            targetOverlapEdgeIndexPairList.emplace_back(std::make_pair(i, i + 1));
        }
    }

    if (targetOverlapEdgeIndexPairList.size() > 0)
    {
        return true;
    }

    // 一致する頂点を検索する
    std::vector<std::pair<size_t, size_t>> equalPointIndexPairList;
    for (int i = 0; i < targetPolygon.outer().size() - 1; i++)
    {
        for (int j = 0; j < searchPolygon.outer().size() - 1; j++)
        {
            // 一致する頂点があった場合
            if (targetPolygon.outer()[i].IsRoundEqual(searchPolygon.outer()[j]))
            {
                equalPointIndexPairList.emplace_back(std::make_pair(i, j));
            }
        }
    }

    if (equalPointIndexPairList.size() < 2)
    {
        return false;
    }

    // 一致する頂点の組み合わせを総当たりで検索
    int maxEqualPointCount = 0;
    int minPolylineSize = 9999;
    std::pair<size_t, size_t> overlapIndexPair;
    for (int i = 0; i < equalPointIndexPairList.size(); i++)
    {
        for (int j = 0; j < equalPointIndexPairList.size(); j++)
        {
            if (i == j)
            {
                continue;
            }

            size_t startTargetIndex = equalPointIndexPairList[i].first;
            size_t endTargetIndex = equalPointIndexPairList[j].first;
            size_t startSearchIndex = equalPointIndexPairList[i].second;
            size_t endSearchIndex = equalPointIndexPairList[j].second;

            // target側の2本の重複候補辺を取得
            Boost3DHashPolyline targetPolyline1;
            Boost3DHashPolyline targetPolyline2;
            int targetEqualPointCount1 = 0;
            int targetEqualPointCount2 = 0;
            bool isPolyline2 = false;

            for (int ii = 0; ii < targetPolygon.outer().size() - 1; ii++)
            {
                size_t idx = (ii + startTargetIndex) % (targetPolygon.outer().size() - 1);

                if (isPolyline2)
                {
                    for (auto indexPair : equalPointIndexPairList)
                    {
                        if (idx == indexPair.first)
                        {
                            targetEqualPointCount2++;
                            break;
                        }
                    }

                    targetPolyline2.emplace_back(targetPolygon.outer()[idx]);

                    if (idx == startTargetIndex)
                    {
                        break;
                    }
                }
                else
                {
                    for (auto indexPair : equalPointIndexPairList)
                    {
                        if (idx == indexPair.first)
                        {
                            targetEqualPointCount1++;
                            break;
                        }
                    }

                    targetPolyline1.emplace_back(targetPolygon.outer()[idx]);

                    if (idx == endTargetIndex)
                    {
                        targetPolyline2.emplace_back(targetPolygon.outer()[idx]);

                        isPolyline2 = true;
                    }
                }
            }

            if ((targetEqualPointCount1 > maxEqualPointCount)
                || ((targetEqualPointCount1 >= maxEqualPointCount) && (targetPolyline1.size() < minPolylineSize)))
            {
                maxEqualPointCount = targetEqualPointCount1;
                minPolylineSize = targetPolyline1.size();
                overlapIndexPair = std::make_pair(startTargetIndex, endTargetIndex);
            }

            if ((targetEqualPointCount2 > maxEqualPointCount)
                || ((targetEqualPointCount2 >= maxEqualPointCount) && (targetPolyline2.size() < minPolylineSize)))
            {
                maxEqualPointCount = targetEqualPointCount2;
                minPolylineSize = targetPolyline2.size();
                overlapIndexPair = std::make_pair(endTargetIndex, startTargetIndex);
            }
        }
    }

    targetOverlapEdgeIndexPairList.emplace_back(overlapIndexPair);

    return true;
}

/*!
 * @brief 2つのポリゴンが重畳しているかどうか判別する
 * @param targetPolygon                     入力ポリゴン1
 * @param searchPolygon                     入力ポリゴン2
 * @param targetOverlapEdgeIndexPairList    重畳している辺の始点と終点のペアリスト
 * @return 処理結果
 * @retval true     重畳している
 * @retval false    重畳していない
*/
bool CNetworkCreator::IsOverlapPolygons(
    Boost3DHashPolygon& targetPolygon,
    Boost3DHashMultiPolygon& searchPolygonList,
    std::vector<std::pair<size_t, size_t>>& targetOverlapEdgeIndexPairList)
{
    if (bg::is_empty(targetPolygon)
        || bg::is_empty(searchPolygonList))
    {
        return false;
    }

    for (auto searchPolygon : searchPolygonList)
    {
        std::vector<std::pair<size_t, size_t>> tmpTargetOverlapEdgeIndexPairList;

        if (IsOverlapPolygons(targetPolygon, searchPolygon, tmpTargetOverlapEdgeIndexPairList))
        {
            for (auto tmpTargetOverlapEdgeIndexPair : tmpTargetOverlapEdgeIndexPairList)
            {
                targetOverlapEdgeIndexPairList.emplace_back(tmpTargetOverlapEdgeIndexPair);
            }
        }
    }

    return true;
}

/*!
 * @brief ポリラインの間引き処理
 * @param src       入力ポリライン
 * @return 間引き後のマルチポリライン
*/
Boost3DHashPolyline CNetworkCreator::ThinOutVerticesOfPolyline(Boost3DHashPolyline src, double interval)
{
    Boost3DHashPolyline dst = Boost3DHashPolyline(src);

    // 始点と終点以外の点が存在しない場合は
    // 間引き不可能
    if (dst.size() <= 2)
    {
        return dst;
    }

    for (int i = 1; i < dst.size() - 1; i++)
    {
        Boost3DPointHash beforePoint    = dst[i - 1];
        Boost3DPointHash currentPoint   = dst[i];
        Boost3DPointHash afterPoint     = dst[i + 1];

        CVector2D toBeforeVec = CVector2D(beforePoint.x(), beforePoint.y()) - CVector2D(currentPoint.x(), currentPoint.y());
        CVector2D toAfterVec = CVector2D(afterPoint.x(), afterPoint.y()) - CVector2D(currentPoint.x(), currentPoint.y());

        // 間引き間隔の調整
        if (CEpsUtil::Greater(toBeforeVec.Length(), interval))
        {
            continue;
        }

        if(IsParallel(toBeforeVec, toAfterVec, -1))
        {
            dst.erase(dst.begin() + i);
            i--;
        }
    }

    // 間引き後のポリラインが不正だった場合は
    // 入力ポリラインをそのまま返す
    if (bg::is_empty(dst)
        || bg::is_valid(dst) == false
        || dst.size() < 2)
    {
        return src;
    }

    return dst;
}

/*!
 * @brief ポリラインの両端をポリゴンにぶつかるまで延長する
 * @param inputPolyline             入力ポリライン
 * @param collisionTargetPolygons   衝突対象ポリゴン群
 * @return 延長後のマルチポリライン
*/
Boost3DHashPolyline CNetworkCreator::ExtendPolylineUntilPolygon(Boost3DHashPolyline& inputPolyline, Boost3DHashMultiPolygon& collisionTargetPolygons)
{
    Boost3DHashPolyline outputPolyline = inputPolyline;

    if (bg::is_empty(inputPolyline)
        || bg::is_valid(inputPolyline) == false
        || inputPolyline.size() < 2)
    {
        return inputPolyline;
    }

    // 中心線両端の座標とベクトル
    Boost3DPointHash startPoint = *(outputPolyline.cbegin());
    Boost3DPointHash startNextPoint = *(outputPolyline.cbegin() + 1);
    Boost3DPointHash endPoint = *(outputPolyline.cend() - 1);
    Boost3DPointHash endNextPoint = *(outputPolyline.cend() - 2);

    CVector2D startPointVec2d = CVector2D(startPoint.x(), startPoint.y());
    CVector2D start2NextVec2d = CVector2D(startPoint.x() - startNextPoint.x(), startPoint.y() - startNextPoint.y());
    CVector2D endPointVec2d = CVector2D(endPoint.x(), endPoint.y());
    CVector2D end2NextVec2d = CVector2D(endPoint.x() - endNextPoint.x(), endPoint.y() - endNextPoint.y());

    start2NextVec2d.Normalize();
    end2NextVec2d.Normalize();

    // 車道ポリゴンの周囲ポリライン
    std::vector<std::pair<Boost3DPointHash, Boost3DPointHash>> polygonPerimeterSegmentPointPairList; // 始点、終点

    for (auto polygon : collisionTargetPolygons)
    {
        Boost3DHashPolyline perimeterPolyline;

        for (auto point : polygon.outer())
        {
            perimeterPolyline.emplace_back(point);
        }

        // CVector2d型に変換して保存
        for (int i = 0; i < perimeterPolyline.size() - 1; i++)
        {
            Boost3DPointHash firstPoint = perimeterPolyline[i];
            Boost3DPointHash secondPoint = perimeterPolyline[i + 1];

            polygonPerimeterSegmentPointPairList.emplace_back(std::make_pair(firstPoint, secondPoint));
        }
    }

    // 交点の内、一番近いものに向かって延伸する
    Boost3DPointHash frontCrossPoint;
    Boost3DPointHash backCrossPoint;
    double frontT = 9999;
    double backT = 9999;
    bool isCrossFront = false;
    bool isCrossBack = false;

    // 中心線始点の延伸
    for (auto pair : polygonPerimeterSegmentPointPairList)
    {
        CVector2D posVec2d = CVector2D(pair.first.x(), pair.first.y());
        CVector2D vecVec2d = CVector2D(pair.second.x() - pair.first.x(), pair.second.y() - pair.first.y());

        CVector2D crossPosVec2d;
        bool isOnLine1 = false;
        bool isOnLine2 = false;
        double t = 0.0;
        double s = 0.0;
        if (CGeoUtil::GetCrossPos(start2NextVec2d,
            startPointVec2d,
            vecVec2d,
            posVec2d,
            crossPosVec2d,
            isOnLine1,
            isOnLine2,
            t,
            s)
            && isOnLine2
            && CEpsUtil::GreaterEqual(t, 0.0)
            && CEpsUtil::Greater(s, 0.0)
            && CEpsUtil::LessEqual(s, 1.0))
        {
            // 交点をBoost3DPointHash型に変換
            // 高さも計算する
            double crossPosZ = pair.first.z() + ((pair.second.z() - pair.first.z()) * s);
            Boost3DPointHash crossPos = Boost3DPointHash(
                crossPosVec2d.x,
                crossPosVec2d.y,
                crossPosZ);

            // 追加する頂点が重複する場合は追加しない
            if(crossPos.IsRoundEqual(outputPolyline.front()))
            {
                continue;
            }

            if (CEpsUtil::Less(t, frontT))
            {
                frontCrossPoint = crossPos;
                isCrossFront = true;
            }
        }
    }

    // 中心線終点の延伸
    for (auto pair : polygonPerimeterSegmentPointPairList)
    {
        CVector2D posVec2d = CVector2D(pair.first.x(), pair.first.y());
        CVector2D vecVec2d = CVector2D(pair.second.x() - pair.first.x(), pair.second.y() - pair.first.y());

        CVector2D crossPosVec2d;
        bool isOnLine1 = false;
        bool isOnLine2 = false;
        double t = 0.0;
        double s = 0.0;
        if (CGeoUtil::GetCrossPos(end2NextVec2d,
            endPointVec2d,
            vecVec2d,
            posVec2d,
            crossPosVec2d,
            isOnLine1,
            isOnLine2,
            t,
            s)
            && isOnLine2
            && CEpsUtil::GreaterEqual(t, 0.0)
            && CEpsUtil::Greater(s, 0.0)
            && CEpsUtil::LessEqual(s, 1.0))
        {
            // 交点をBoost3DPointHash型に変換
            // 高さも計算する
            double crossPosZ = pair.first.z() + ((pair.second.z() - pair.first.z()) * s);
            Boost3DPointHash crossPos = Boost3DPointHash(
                crossPosVec2d.x,
                crossPosVec2d.y,
                crossPosZ);

            // 追加する頂点が重複する場合は追加しない
            if(crossPos.IsRoundEqual(outputPolyline.back()))
            {
                continue;
            }

            if (CEpsUtil::Less(t, backT))
            {
                backCrossPoint = crossPos;
                isCrossBack = true;
            }
        }
    }

    // 交点を追加する
    if (isCrossFront)
    {
        outputPolyline.insert(outputPolyline.cbegin(), frontCrossPoint);
    }

    if (isCrossBack)
    {
        outputPolyline.insert(outputPolyline.cend(), backCrossPoint);
    }

    if (bg::is_empty(outputPolyline)
        || bg::is_valid(outputPolyline) == false
        || outputPolyline.size() < 2)
    {
        return inputPolyline;
    }

    return outputPolyline;
}

/*!
 * @brief ポリラインの両端がポリゴンからはみ出さないようにトリミングする
 * @param inputPolyline             入力ポリライン
 * @param collisionTargetPolygons   衝突対象ポリゴン群
 * @return トリミング後のマルチポリライン
*/
Boost3DHashPolyline CNetworkCreator::TrimPolylineUntilPolygon(Boost3DHashPolyline& inputPolyline, Boost3DHashMultiPolygon& collisionTargetPolygons)
{
    Boost3DHashPolyline outputPolyline = inputPolyline;

    if (bg::is_empty(inputPolyline)
        || bg::is_valid(inputPolyline) == false
        || inputPolyline.size() < 2)
    {
        return inputPolyline;
    }

    // 車道ポリゴンの周囲ポリライン
    std::vector<std::pair<Boost3DPointHash, Boost3DPointHash>> polygonPerimeterSegmentPointPairList; // 始点、終点

    for (auto polygon : collisionTargetPolygons)
    {
        Boost3DHashPolyline perimeterPolyline;

        for (auto point : polygon.outer())
        {
            perimeterPolyline.emplace_back(point);
        }

        // CVector2d型に変換して保存
        for (int i = 0; i < perimeterPolyline.size() - 1; i++)
        {
            Boost3DPointHash firstPoint = perimeterPolyline[i];
            Boost3DPointHash secondPoint = perimeterPolyline[i + 1];

            polygonPerimeterSegmentPointPairList.emplace_back(std::make_pair(firstPoint, secondPoint));
        }
    }

    // 中心線始点のトリミング
    for (int i = 0; i < outputPolyline.size() - 1; i++)
    {
        // 対象の点がポリゴンに入っていれば判定終了
        if (CoveredByPolygonsIgnoreZ(outputPolyline[i], collisionTargetPolygons))
        {
            break;
        }

        // 対象の次の点もポリゴン外であればスキップ
        if (CoveredByPolygonsIgnoreZ(outputPolyline[i + 1], collisionTargetPolygons) == false)
        {
            continue;
        }

        // セグメントがポリゴンと交差する箇所を求める
        CVector2D polylineSegmentPosVec2d = CVector2D(outputPolyline[i].x(), outputPolyline[i].y());
        CVector2D polylineSegmentVecVec2d = CVector2D(outputPolyline[i + 1].x() - outputPolyline[i].x(), outputPolyline[i + 1].y() - outputPolyline[i].y());
        Boost3DPointHash crossPoint;
        for (auto pair : polygonPerimeterSegmentPointPairList)
        {
            CVector2D polygonSegmentPosVec2d = CVector2D(pair.first.x(), pair.first.y());
            CVector2D polygonSegmentVecVec2d = CVector2D(pair.second.x() - pair.first.x(), pair.second.y() - pair.first.y());

            CVector2D crossPosVec2d;
            bool isOnLine1 = false;
            bool isOnLine2 = false;
            double t = 0.0;
            double s = 0.0;

            if (CGeoUtil::GetCrossPos(
                polylineSegmentVecVec2d,
                polylineSegmentPosVec2d,
                polygonSegmentVecVec2d,
                polygonSegmentPosVec2d,
                crossPosVec2d,
                isOnLine1,
                isOnLine2,
                t,
                s)
                && isOnLine1
                && isOnLine2
                && CEpsUtil::GreaterEqual(t, 0.0)
                && CEpsUtil::Less(t, 1.0)
                && CEpsUtil::GreaterEqual(s, 0.0)
                && CEpsUtil::Less(s, 1.0))
            {
                crossPoint = Boost3DPointHash(
                    outputPolyline[i].x() + t * (outputPolyline[i + 1].x() - outputPolyline[i].x()),
                    outputPolyline[i].y() + t * (outputPolyline[i + 1].y() - outputPolyline[i].y()),
                    outputPolyline[i].z() + t * (outputPolyline[i + 1].z() - outputPolyline[i].z())
                );

                // はみ出している分を削除
                for (int j = 0; j <= i; j++)
                {
                    outputPolyline.erase(outputPolyline.begin());
                }

                // ポリゴンとの交点を追加する
                if (outputPolyline.front().IsRoundEqual(crossPoint) == false)
                {
                    outputPolyline.insert(outputPolyline.begin(), crossPoint);
                }

                break;
            }
        }

        // トリミング処理をしたら終了
        break;
    }

    // 中心線終点のトリミング
    for (int i = outputPolyline.size() - 1; i > 0; i--)
    {
        // 対象の点がポリゴンに入っていれば判定終了
        if (CoveredByPolygonsIgnoreZ(outputPolyline[i], collisionTargetPolygons))
        {
            break;
        }

        // 対象の次の点もポリゴン外であればスキップ
        if (CoveredByPolygonsIgnoreZ(outputPolyline[i - 1], collisionTargetPolygons) == false)
        {
            continue;
        }

        // セグメントがポリゴンと交差する箇所を求める
        CVector2D polylineSegmentPosVec2d = CVector2D(outputPolyline[i].x(), outputPolyline[i].y());
        CVector2D polylineSegmentVecVec2d = CVector2D(outputPolyline[i - 1].x() - outputPolyline[i].x(), outputPolyline[i - 1].y() - outputPolyline[i].y());
        Boost3DPointHash crossPoint;
        for (auto pair : polygonPerimeterSegmentPointPairList)
        {
            CVector2D polygonSegmentPosVec2d = CVector2D(pair.first.x(), pair.first.y());
            CVector2D polygonSegmentVecVec2d = CVector2D(pair.second.x() - pair.first.x(), pair.second.y() - pair.first.y());

            CVector2D crossPosVec2d;
            bool isOnLine1 = false;
            bool isOnLine2 = false;
            double t = 0.0;
            double s = 0.0;

            if (CGeoUtil::GetCrossPos(
                polylineSegmentVecVec2d,
                polylineSegmentPosVec2d,
                polygonSegmentVecVec2d,
                polygonSegmentPosVec2d,
                crossPosVec2d,
                isOnLine1,
                isOnLine2,
                t,
                s)
                && isOnLine1
                && isOnLine2
                && CEpsUtil::GreaterEqual(t, 0.0)
                && CEpsUtil::Less(t, 1.0)
                && CEpsUtil::GreaterEqual(s, 0.0)
                && CEpsUtil::Less(s, 1.0))
            {
                crossPoint = Boost3DPointHash(
                    outputPolyline[i].x() + t * (outputPolyline[i - 1].x() - outputPolyline[i].x()),
                    outputPolyline[i].y() + t * (outputPolyline[i - 1].y() - outputPolyline[i].y()),
                    outputPolyline[i].z() + t * (outputPolyline[i - 1].z() - outputPolyline[i].z())
                );

                // はみ出している分を削除
                for (int j = outputPolyline.size() - 1; j >= i; j--)
                {
                    outputPolyline.erase(outputPolyline.end() - 1);
                }

                // ポリゴンとの交点を追加する
                if (outputPolyline.back().IsRoundEqual(crossPoint) == false)
                {
                    outputPolyline.insert(outputPolyline.end(), crossPoint);
                }

                break;
            }
        }

        // トリミング処理をしたら終了
        break;
    }

    if (bg::is_empty(outputPolyline)
        || bg::is_valid(outputPolyline) == false
        || outputPolyline.size() < 2)
    {
        return inputPolyline;
    }

    return outputPolyline;

}

/*!
 * @brief ポリゴン内にポイントが入っているかどうかの確認
 * @brief 高さ情報を無視する
 * @param inputPoint        入力ポイント
 * @param inputPolygons     入力ポリゴン群
 * @return 処理結果
 * @retval true     入っている
 * @retval false    入っていない
*/
bool CNetworkCreator::CoveredByPolygonsIgnoreZ(
    Boost3DPointHash& inputPoint,
    Boost3DHashMultiPolygon& inputPolygons)
{
    if (bg::is_empty(inputPoint)
        || bg::is_empty(inputPolygons))
    {
        return false;
    }

    // 高さ情報を落としたGeometryに変換
    Boost3DPointHash tmpInputPoint = Boost3DPointHash(inputPoint.x(), inputPoint.y(), 0);
    Boost3DHashMultiPolygon tmpInputPolygons;
    for (auto polygon : inputPolygons)
    {
        Boost3DHashPolygon tmpPolygon;
        for (auto point : polygon.outer())
        {
            tmpPolygon.outer().emplace_back(Boost3DPointHash(point.x(), point.y(), 0));
        }
        tmpInputPolygons.emplace_back(tmpPolygon);
    }

    // ポリゴンごとに判定
    // ポリゴン一つでも入っていたらtrue
    bool isCoverdBy = false;
    for (auto polygon : tmpInputPolygons)
    {
        if (bg::covered_by(tmpInputPoint, polygon))
        {
            isCoverdBy = true;
            break;
        }
    }

    return isCoverdBy;
}

/*!
 * @brief ポリゴン内にポイントが入っているかどうかの確認
 * @brief 高さ情報を無視する
 * @param inputPoint        入力ポイント
 * @param inputPolygons     入力ポリゴン群
 * @return 処理結果
 * @retval true     入っている
 * @retval false    入っていない
*/
bool CNetworkCreator::CoveredByPolygonsIgnoreZ(
    Boost3DMultiPointHashs& inputPoints,
    Boost3DHashMultiPolygon& inputPolygons)
{
    bool isCoverdBy = true;

    for (auto point : inputPoints)
    {
        // ポイントが一つでもはみ出していたらfalse
        if (CoveredByPolygonsIgnoreZ(point, inputPolygons) == false)
        {
            isCoverdBy = false;
        }
    }

    return isCoverdBy;
}

/*!
 * @brief ポリゴンからポリラインの途中がはみ出しているかどうかの確認
 * @param inputPolyline     入力ポリライン
 * @param inputPolygons     入力ポリゴン群
 * @return 処理結果
 * @retval true     はみ出している
 * @retval false    はみ出していない
*/
bool CNetworkCreator::CheckPolylineProtrudeFromPolygon(
    Boost3DHashPolyline& inputPolyline,
    Boost3DHashMultiPolygon& inputPolygons)
{
    // 入力ポリラインのチェック
    if (bg::is_empty(inputPolyline)
        || bg::is_valid(inputPolyline) == false
        || inputPolyline.size() < 2)
    {
        // 正確には、はみ出しているかどうか不明
        return true;
    }

    // 入力ポリゴンのチェック
    if (bg::is_empty(inputPolygons))
    {
        // 正確には、はみ出しているかどうか不明
        return true;
    }

    // ポリラインのサンプリング
    auto sampledPolyline = CBoostGeoUtil::Sampling(inputPolyline, 1.0);

    int currProtrudeState = 0;
    int prevProtrudeState = 0;
    int beforePrevProtrudeState = 0;

    for (auto point : sampledPolyline)
    {
        for (auto polygon : inputPolygons)
        {
            // ポリラインの頂点がポリゴン外に出ている場合
            // はみ出し判定とする
            if (bg::covered_by(point, polygon) == false)
            {
                currProtrudeState = -1;
            }
            else
            {
                // どれかのポリゴンに入っていれば
                // 判定終了
                currProtrudeState = 1;
                break;
            }
        }

        if (currProtrudeState == 1
            && prevProtrudeState == -1
            && beforePrevProtrudeState == 1)
        {
            // 途中ではみ出している場合
            return true;
        }

        if (currProtrudeState != prevProtrudeState)
        {
            beforePrevProtrudeState = prevProtrudeState;
            prevProtrudeState = currProtrudeState;
        }
    }

    if (currProtrudeState != 1
        && prevProtrudeState != 1
        && beforePrevProtrudeState != 1)
    {
        // 一度もポリゴンに入っていない場合
        return true;
    }

    return false;
}

/*!
 * @brief ポリゴンからポリラインの途中がはみ出しているかどうかの確認
 * @param inputPolylineList     入力ポリライン群
 * @param inputPolygons         入力ポリゴン群
 * @return 処理結果
 * @retval true     はみ出している
 * @retval false    はみ出していない
*/
bool CNetworkCreator::CheckPolylineProtrudeFromPolygon(
    Boost3DHashMultiLines& inputPolylineList,
    Boost3DHashMultiPolygon& inputPolygons)
{
    for (auto polyline : inputPolylineList)
    {
        // どれか一つはみ出していたらtrue
        if (CheckPolylineProtrudeFromPolygon(polyline, inputPolygons))
        {
            return true;
        }
    }

    return false;
}

/*!
 * @brief 同数の複数点ペアリング
 * @param[in ] pointList1      複数点セット1
 * @param[in ] pointList2      複数点セット2
 * @param[out] pairingList     ペアリング結果（ペア同士のインデックス）
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::PairMultiplePoints(
    Boost3DMultiPointHashs& pointList1,
    Boost3DMultiPointHashs& pointList2,
    std::vector<std::pair<size_t, size_t>>& pairingList)
{
    if (pointList1.size() != pointList2.size())
    {
        return false;
    }

    // 各組合せと合計距離のリスト
    std::vector<std::pair<std::vector<size_t>, double>> combinationList;

    // 点を総当たり設定
    std::vector<size_t> indices;
    for (size_t i = 0; i < pointList2.size(); i++)
    {
        indices.emplace_back(i);
    }

    // 総当たりで距離を算出
    do {
        double totalDistance = 0.0;
        for (size_t i = 0; i < pointList1.size(); i++)
        {
            totalDistance += bg::distance(pointList1[i], pointList2[indices[i]]);
        }

        combinationList.emplace_back(std::make_pair(indices, totalDistance));
    } while (std::next_permutation(indices.begin(), indices.end()));

    // 最短距離を見つける
    auto minCombination = *std::min_element(combinationList.cbegin(), combinationList.cend(),
        [](const auto& lhs, const auto& rhs) {return lhs.second < rhs.second; });

    // 計算結果を保存する
    for (size_t i = 0; i < pointList1.size(); i++)
    {
        pairingList.emplace_back(std::make_pair(i, minCombination.first[i]));
    }

    return true;
}

/*!
 * @brief 異なる本数のポリラインを接続する
 * @param inputPolylinePtrList1     入力ポリライン群1
 * @param inputPolylinePtrList2     入力ポリライン群2
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::ConnectCenterLineOfDifferentCount(
    std::vector<Boost3DHashPolyline*> inputPolylinePtrList1,
    std::vector<Boost3DHashPolyline*> inputPolylinePtrList2)
{
    if (inputPolylinePtrList1.size() == inputPolylinePtrList2.size())
    {
        return false;
    }

    if (inputPolylinePtrList1.size() > inputPolylinePtrList2.size())
    {
        inputPolylinePtrList1.swap(inputPolylinePtrList2);
    }

    for (auto targetPolylinePtr : inputPolylinePtrList1)
    {
        // 入力ポリライン1の接続する点を決定する
        double dist2TargetPolylineFront = 0.0;
        double dist2TargetPolylineBack = 0.0;

        for (auto anotherPolylinePtr : inputPolylinePtrList2)
        {
            dist2TargetPolylineFront += bg::distance(targetPolylinePtr->front(), *anotherPolylinePtr);
            dist2TargetPolylineBack += bg::distance(targetPolylinePtr->back(), *anotherPolylinePtr);
        }

        if (CEpsUtil::LessEqual(dist2TargetPolylineFront, dist2TargetPolylineBack))
        {
            auto frontPointIt = targetPolylinePtr->cbegin();
            auto nextFrontPointIt = targetPolylinePtr->cbegin() + 1;

            // 分岐する点を決める
            CVector3D front2NextVec3d = CVector3D(nextFrontPointIt->x(), nextFrontPointIt->y(), nextFrontPointIt->z())
                - CVector3D(frontPointIt->x(), frontPointIt->y(), frontPointIt->z());

            if (CEpsUtil::Greater(front2NextVec3d.Length(), 1.0))
            {
                // 接続する点と一つ後の点が 1m 以上ある場合は
                // 1m位置に頂点を追加する
                front2NextVec3d.Normalize();
                Boost3DPointHash addPoint = Boost3DPointHash(frontPointIt->x() + front2NextVec3d.x, frontPointIt->y() + front2NextVec3d.y, frontPointIt->z() + front2NextVec3d.z);
                targetPolylinePtr->insert(targetPolylinePtr->cbegin() + 1, addPoint);
            }
            else
            {
                // 接続する点と一つ後の点が 1m 未満の場合は
                // 一つ後の点を分岐する点にする
                // (特に処理は無し)
            }

            // 始点を削除
            targetPolylinePtr->erase(targetPolylinePtr->cbegin());

            // 各入力ポリライン2と接続する
            for (auto anotherPolylinePtr : inputPolylinePtrList2)
            {
                // 分岐する点に近い入力ポリライン2の頂点を接続点にする
                bool isFront = CEpsUtil::LessEqual(
                    bg::distance(targetPolylinePtr->front(), anotherPolylinePtr->front()),
                    bg::distance(targetPolylinePtr->front(), anotherPolylinePtr->back())
                );

                if (isFront)
                {
                    anotherPolylinePtr->insert(anotherPolylinePtr->cbegin(), targetPolylinePtr->front());
                }
                else
                {
                    anotherPolylinePtr->insert(anotherPolylinePtr->cend(), targetPolylinePtr->front());
                }
            }
        }
        else
        {
            auto backPointIt = targetPolylinePtr->cend() - 1;
            auto prevBackPointIt = targetPolylinePtr->cend() - 2;

            // 分岐する点を決める
            CVector3D back2PrevVec3d = CVector3D(prevBackPointIt->x(), prevBackPointIt->y(), prevBackPointIt->z())
                - CVector3D(backPointIt->x(), backPointIt->y(), backPointIt->z());

            if (CEpsUtil::Greater(back2PrevVec3d.Length(), 1.0))
            {
                // 接続する点と一つ前の点が 1m 以上ある場合は
                // 1m位置に頂点を追加する
                back2PrevVec3d.Normalize();
                Boost3DPointHash addPoint = Boost3DPointHash(backPointIt->x() + back2PrevVec3d.x, backPointIt->y() + back2PrevVec3d.y, backPointIt->z() + back2PrevVec3d.z);
                targetPolylinePtr->insert(targetPolylinePtr->cend() - 1, addPoint);
            }
            else
            {
                // 接続する点と一つ前の点が 1m 未満の場合は
                // 一つ後の点を分岐する点にする
                // (特に処理は無し)
            }

            // 始点を削除
            targetPolylinePtr->erase(targetPolylinePtr->cend() - 1);

            // 各入力ポリライン2と接続する
            for (auto anotherPolylinePtr : inputPolylinePtrList2)
            {
                // 分岐する点に近い入力ポリライン2の頂点を接続点にする
                bool isFront = CEpsUtil::LessEqual(
                    bg::distance(targetPolylinePtr->back(), anotherPolylinePtr->front()),
                    bg::distance(targetPolylinePtr->back(), anotherPolylinePtr->back())
                );

                if (isFront)
                {
                    anotherPolylinePtr->insert(anotherPolylinePtr->cbegin(), targetPolylinePtr->back());
                }
                else
                {
                    anotherPolylinePtr->insert(anotherPolylinePtr->cend(), targetPolylinePtr->back());
                }
            }
        }
    }

    return true;
}

/*!
 * @brief 入力ベクトルの水平確認
 * @param vec1              入力ベクトル1
 * @param vec2              入力ベクトル2
 * @param checkDirection    確認する2ベクトルの方向
 * @retval 0    順方向か逆方向で水平か確認
 * @retval 正   順方向のみ水平か確認（逆方向は水平でもfalse）
 * @retval 負   逆方向のみ水平か確認（順方向は水平でもfalse）
 * @return 処理結果
 * @retval true         水平
 * @retval false        水平でない
*/
bool CNetworkCreator::IsParallel(
    const CVector2D& vec1,
    const CVector2D& vec2,
    int checkDirection)
{
    double angle = CGeoUtil::Angle(vec1, vec2);
    bool isForward = CEpsUtil::GreaterEqual(angle, MIN_ZERO_ANGLE) && CEpsUtil::LessEqual(angle, MAX_ZERO_ANGLE);
    bool isReverse = CEpsUtil::GreaterEqual(angle, MIN_PARALLEL) && CEpsUtil::LessEqual(angle, MAX_PARALLEL);

    if (checkDirection > 0)
    {
        return isForward;
    }
    else if(checkDirection < 0)
    {
        return isReverse;
    }

    // どちらか水平であればtrue
    return isForward || isReverse;
}

// 車道ネットワークデータの出力
bool CNetworkCreator::OutputRoadwayNetwork(
    const std::string &strShpOutputFolder,
    const std::string &strGeoJsonOutputFolder,
    const int nJPZone,
    const bool bCreateSHP,
    const bool bCreateGeoJSON,
    const double dLod3Detail)
{
    if (!bCreateGeoJSON && !bCreateSHP)
        return false;

    CNetwork nw(CNetwork::NETWORK_DATA_TYPE::ROADWAY, dLod3Detail);
    // ネットワークデータの作成
    nw.Add(m_tranRoadData);

    // 出力ファイルの作成
    CNetwork::OUTPUT_FILE_TYPE type;
    if (bCreateGeoJSON && bCreateSHP)
    {
        type = CNetwork::OUTPUT_FILE_TYPE::BOTH;
    }
    else if (bCreateGeoJSON)
    {
        type = CNetwork::OUTPUT_FILE_TYPE::GEOJSON;
    }
    else
    {
        type = CNetwork::OUTPUT_FILE_TYPE::SHP;
    }

    bool isUseZ = CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3 ? true : false;
    nw.OutputNetworkData(strShpOutputFolder, strGeoJsonOutputFolder, nJPZone, isUseZ, type);

    return true;
}

// 横断歩道の中心線作成
void CNetworkCreator::CreateCenterLineOfPedestrianCrossing()
{
    for (auto &crossing : m_pedestrianCrossingData)
    {
        CFurnitureDataUtil::CreateCenterLineOfPedestrianCrossing(crossing);
    }
}

/*!
 * @brief 歩道のエッジ検出
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::DetectEdgeOfFootpath()
{
    for (auto tranRoadData : m_tranRoadData)
    {
        Boost3DHashMultiPolygon targetPolygonList;
        Boost3DHashMultiPolygon neighborPolygonList;
        Boost3DHashMultiPolygon polygonList;            // 注目道路ポリゴンの融合用
        Boost3DHashMultiPolygon footpathPolygonList;    // 歩道部ポリゴン
        Boost3DHashMultiPolygon plantPolygonList;       // LOD3.2以上の植栽用

        // 使用するLODのデータを準備
        switch (m_iLod)
        {
        case 2:
            for (auto lod2 : tranRoadData->m_lod2List)
            {
                polygonList.emplace_back(lod2.m_boostGeometry);
                // 歩道部のみ
                if (lod2.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH))
                    targetPolygonList.emplace_back(lod2.m_boostGeometry);
            }

            for (std::shared_ptr<CTranRoadData> neighborTranRoadDataPtrList : tranRoadData->m_neighborRoadPtr)
            {
                for (auto lod2 : neighborTranRoadDataPtrList->m_lod2List)
                {
                    neighborPolygonList.emplace_back(lod2.m_boostGeometry);
                }
            }
            break;
        case 3:
            // 歩道部ポリゴン or 歩道部+植栽ポリゴン(LOD3.2以上)の取得
            targetPolygonList = CTranRoadDataUtil::GetFootpathAndPlantsPolygon(*tranRoadData);
            // 注目道路ポリゴン全体範囲作成用ポリゴンの取得
            for (auto lod3 : tranRoadData->m_lod3List)
            {
                polygonList.emplace_back(lod3.m_boostGeometry);
            }
            // 近傍道路ポリゴンの取得
            for (std::shared_ptr<CTranRoadData> neighborTranRoadDataPtrList : tranRoadData->m_neighborRoadPtr)
            {
                for (auto lod3 : neighborTranRoadDataPtrList->m_lod3List)
                {
                    neighborPolygonList.emplace_back(lod3.m_boostGeometry);
                }
            }
            break;
        default:
            break;
        }

        if (targetPolygonList.size() == 0)
        {
            // 歩道部ポリゴンがない場合
            continue;
        }

        // 注目道路全体のポリゴンを作成(LOD1相当)
        // 理想はLOD1相当のポリゴン1つに融合されることだが、
        // データによって融合が失敗する場合を考慮して、最大面積のポリゴンを選択する
        Boost3DHashMultiPolygon dissolve = CGDALUtil::GetInstance()->Dissolve(polygonList);
        for (auto &polygon : dissolve)
        {
            polygon.inners().clear();   // 穴は不要なため削除
            BoostPolygon poly = CBoostGeoUtil::Conv(polygon);
        }

        // ポリゴン毎にエッジ線を検出
        for (Boost3DHashPolygon targetPolygon : targetPolygonList)
        {
            if (bg::is_empty(targetPolygon))
            {
                continue;
            }

            // 辺の重複リスト
            std::vector<bool> isEdgeList;
            isEdgeList.resize(targetPolygon.outer().size() - 1, true);

            // 近傍ポリゴンを総検索
            for (auto neighborPolygon : neighborPolygonList)
            {
                if (bg::is_empty(neighborPolygon))
                {
                    continue;
                }

                std::vector<std::pair<size_t, size_t>> targetOverlapEdgeIndexPairList;
                if (IsOverlapPolygons(targetPolygon, neighborPolygon, targetOverlapEdgeIndexPairList))
                {
                    for (auto overlapEdgeIdxPair : targetOverlapEdgeIndexPairList)
                    {
                        for (size_t idx = overlapEdgeIdxPair.first; idx < overlapEdgeIdxPair.second; idx++)
                        {
                            isEdgeList[idx] = false;
                        }
                    }
                }
            }

            // 辺の重複リストからエッジを作成
            Boost3DHashPolyline edge;
            Boost3DHashMultiLines edges;
            int startIdx = 0;

            // 重複辺の位置をエッジ線作成の開始場所とする
            for (int i = 1; i < isEdgeList.size(); i++)
            {
                if (isEdgeList[i] == true && isEdgeList[i - 1] == false)
                {
                    startIdx = i;
                    break;
                }
            }

            // データの初めと最後が連続したエッジであることを考慮して作成する
            for (int i = 0; i < isEdgeList.size(); i++)
            {
                int targetIdx = (i + startIdx) % isEdgeList.size();
                int prevTargetIdx = (i - 1 + startIdx) % isEdgeList.size();

                if (isEdgeList[targetIdx] == true)
                {
                    edge.emplace_back(targetPolygon.outer()[targetIdx]);
                }
                else if (isEdgeList[prevTargetIdx] == true)
                {
                    edge.emplace_back(targetPolygon.outer()[targetIdx]);
                    edges.emplace_back(edge);
                    edge.clear();
                }
            }

            if (edges.size() == 0)
            {
                // 注目歩道と隣接道路との重畳辺がない場合は、道路縁と車道と歩道の境界線に分割する
                // ex.頂点のみで重畳している場合や道路の途中に歩道が存在し隣接道路がない場合
                // 道路縁と車道と歩道の境界線の切れ目を探索する
                size_t startIdx = 0;
                for (; startIdx < targetPolygon.outer().size(); startIdx++)
                {
                    size_t prev = (startIdx == 0) ? targetPolygon.outer().size() - 2 : startIdx - 1;
                    if (bg::within(targetPolygon.outer().at(prev), dissolve)
                        && !bg::within(targetPolygon.outer().at(startIdx), dissolve))
                        break;  // 車道と歩道の境界線から道路縁に切り替わる箇所
                }
                if (startIdx < targetPolygon.outer().size())
                {
                    Boost3DHashPolyline polyline1, polyline2;
                    size_t idx = startIdx;
                    size_t prev;
                    do
                    {
                        prev = (idx == 0) ? targetPolygon.outer().size() - 2 : idx - 1;

                        if (polyline1.size() < 1
                            || (!bg::within(targetPolygon.outer().at(prev), dissolve)
                                && !bg::within(targetPolygon.outer().at(idx), dissolve)))
                        {
                            polyline1.emplace_back(targetPolygon.outer().at(idx));
                        }
                        else
                        {
                            //LOD1相当のポリゴンの内外の切れ目となる頂点に到達したため1本目のポリラインの作成は終了
                            break;
                        }

                        idx++;
                        if (idx >= targetPolygon.outer().size() - 1)
                            idx = 0;
                    } while (idx != startIdx);

                    // 2本目のポリラインの作成
                    polyline2.emplace_back(targetPolygon.outer().at(prev));
                    while (idx != startIdx)
                    {
                        polyline2.emplace_back(targetPolygon.outer().at(idx));
                        idx++;
                        if (idx >= targetPolygon.outer().size() - 1)
                            idx = 0;
                    }
                    polyline2.emplace_back(targetPolygon.outer().at(startIdx));

                    // エッジの詰め直し
                    edges.clear();
                    edges.emplace_back(polyline1);
                    edges.emplace_back(polyline2);
                }
            }
            else if (edges.size() == 1)
            {
                // 歩道と隣接している道路が1つしかなかった場合
                //　LOD1相当のポリゴン内外の点列に分割する
                Boost3DHashPolyline polyline1, polyline2;
                size_t idx = 0;
                for (; idx < edges[0].size(); idx++)
                {
                    if (idx < 2
                        || (bg::within(edges[0].at(idx - 1), dissolve) && bg::within(edges[0].at(idx), dissolve))
                        || (!bg::within(edges[0].at(idx - 1), dissolve) && !bg::within(edges[0].at(idx), dissolve)))
                    {
                        polyline1.emplace_back(edges[0].at(idx));
                    }
                    else
                    {
                        //LOD1相当のポリゴンの内外の切れ目となる頂点に到達したため1本目のポリラインの作成は終了
                        if (bg::within(edges[0].at(idx - 1), dissolve) && !bg::within(edges[0].at(idx), dissolve))
                            polyline1.emplace_back(edges[0].at(idx));
                        break;
                    }
                }
                // 2本目のポリラインの作成
                if (idx < edges[0].size()
                    && !bg::within(edges[0].at(idx - 1), dissolve)
                    && bg::within(edges[0].at(idx), dissolve))
                    polyline2.emplace_back(edges[0].at(idx - 1));

                for (; idx < edges[0].size(); idx++)
                {
                    polyline2.emplace_back(edges[0].at(idx));
                }

                if (polyline2.size() == 0)
                {
                    // 行き止まり道路で歩道部ポリゴンの頂点が全て線上に位置する場合
                    polyline1.clear();
                    if (edges[0].size() == 4)
                    {
                        polyline1.emplace_back(edges[0].at(0));
                        polyline1.emplace_back(edges[0].at(1));
                        polyline2.emplace_back(edges[0].at(2));
                        polyline2.emplace_back(edges[0].at(3));
                    }
                }

                // エッジの詰め直し
                edges.clear();
                edges.emplace_back(polyline1);
                edges.emplace_back(polyline2);
            }

            // 2辺が取れた場合はエッジとして保存する
            if (edges.size() == 2)
            {
                if (tranRoadData->m_nInOut < 3)
                {
                    // 車道交差部以外はエッジ線の整形処理を行う
                    ShapingEdgeLine(edges[0]);
                    ShapingEdgeLine(edges[1]);
                }
                if (edges[0].size() > 1 && edges[1].size() > 1)
                    tranRoadData->m_footpath.edgePairList.emplace_back(std::make_pair(edges[0], edges[1]));
            }
        }
    }

    return true;
}

/*!
 * @brief エッジ線の整形(歩道のカーブ部分の除去)
 * @param polyline          エッジ線
 * @param dTotalLengthTh    基準方向決定用の探索距離m
 * @param dAngleTh          基準方向との並行確認時の許容角度deg
*/
void CNetworkCreator::ShapingEdgeLine(
    Boost3DHashPolyline &polyline,
    const double dTotalLengthTh,
    const double dAngleTh)
{
    if (polyline.size() < 3)
        return;

    const CVector2D vecEast(1, 0);
    const double dVerticalAngleTh = 80.0;
    CRotateAngleVecDataManager rotMng;      // 進行方向角度の頻度計算用
    double dTotalLength = 0;                // 確認距離総計
    size_t startIdx = 0;                    // 短縮後の始点
    size_t endIdx = polyline.size() - 1;    // 短縮後の終点

    // 始点から探索
    // 基準ベクトルの探索
    for (size_t t = 0; t < polyline.size() - 1; t++)
    {
        CVector2D curr(polyline[t].x(), polyline[t].y());
        CVector2D next(polyline[t + 1].x(), polyline[t + 1].y());
        CVector2D vec = next - curr;
        double dAngle = CGeoUtil::SignedAngle(vecEast, vec);
        double dLength = vec.Length();
        if (CEpsUtil::Greater(dAngle, 90.0))
            dAngle -= 180.0;
        if (CEpsUtil::Less(dAngle, -90.0))
            dAngle += 180.0;
        CRotateAngleVecData d(vec, curr, dAngle, dLength, 1);
        int cnt = static_cast<int>(CUtil::RoundN(dLength, 1) * 10); // 10cm単位で頻度計算
        if (cnt == 0)
            cnt = 1;
        for (int n = 0; n < cnt; n++)
            rotMng.Add(d);
        dTotalLength += dLength;
        if (CEpsUtil::GreaterEqual(dTotalLength, dTotalLengthTh))
            break;
    }
    rotMng.Sort(true);

    if (rotMng.data.size() > 2
        || (rotMng.data.size() == 2
            && CEpsUtil::GreaterEqual(CGeoUtil::Angle(rotMng.data[0].vec, rotMng.data[1].vec), dVerticalAngleTh)))
    {
        CVector2D base;
        base = rotMng.data.back().vec;

        for (size_t t = 0; t < polyline.size() - 1; t++)
        {
            CVector2D curr(polyline[t].x(), polyline[t].y());
            CVector2D next(polyline[t + 1].x(), polyline[t + 1].y());
            CVector2D vec = next - curr;
            double dAngle = CGeoUtil::Angle(base, vec);
            if (CEpsUtil::LessEqual(dAngle, dAngleTh))
            {
                startIdx = t;
                break;
            }
        }
    }

    // 終点側端点から探索
    // 基準ベクトルの探索
    rotMng.data.clear();
    dTotalLength = 0;
    for (size_t t = polyline.size() - 1; t > 0; t--)
    {
        CVector2D curr(polyline[t].x(), polyline[t].y());
        CVector2D next(polyline[t - 1].x(), polyline[t - 1].y());
        CVector2D vec = next - curr;
        double dAngle = CGeoUtil::SignedAngle(vecEast, vec);
        double dLength = vec.Length();
        if (CEpsUtil::Greater(dAngle, 90.0))
            dAngle -= 180.0;
        if (CEpsUtil::Less(dAngle, -90.0))
            dAngle += 180.0;
        CRotateAngleVecData d(vec, curr, dAngle, dLength, 1);
        int cnt = static_cast<int>(CUtil::RoundN(dLength, 1) * 10); // 10cm単位で頻度計算
        if (cnt == 0)
            cnt = 1;
        for (int n = 0; n < cnt; n++)
            rotMng.Add(d);
        dTotalLength += dLength;
        if (CEpsUtil::GreaterEqual(dTotalLength, dTotalLengthTh))
            break;
    }
    rotMng.Sort(true);

    if (rotMng.data.size() > 2
        || (rotMng.data.size() == 2
            && CEpsUtil::GreaterEqual(CGeoUtil::Angle(rotMng.data[0].vec, rotMng.data[1].vec), dVerticalAngleTh)))
    {
        CVector2D base;
        base = rotMng.data.back().vec;
        for (size_t t = polyline.size() - 1; t > 0; t--)
        {
            if (t < startIdx)
                break;

            CVector2D curr(polyline[t].x(), polyline[t].y());
            CVector2D next(polyline[t - 1].x(), polyline[t - 1].y());
            CVector2D vec = next - curr;
            double dAngle = CGeoUtil::Angle(base, vec);

            if (CEpsUtil::LessEqual(dAngle, dAngleTh))
            {
                endIdx = t;
                break;
            }
        }
    }

    //if (startIdx == endIdx)
    //{
    //    // 距離が短い方を削除する
    //    double dLength1 = 0;
    //    for (size_t t = 0; t < startIdx; t++)
    //    {
    //        CVector2D curr(polyline[t].x(), polyline[t].y());
    //        CVector2D next(polyline[t + 1].x(), polyline[t + 1].y());
    //        CVector2D vec = next - curr;
    //        dLength1 += vec.Length();
    //    }
    //    double dLength2 = bg::length(polyline) - dLength1;
    //    if (CEpsUtil::Less(dLength1, dLength2))
    //    {
    //        endIdx = polyline.size() - 1;
    //    }
    //    else
    //    {
    //        startIdx = 0;
    //    }
    //}

    size_t num = endIdx - startIdx + 1;
    if (num > 1 && num < polyline.size())
    {
        // 削除後の頂点が2以上かつ、入力ポリラインの点数より減る場合
        if (endIdx < polyline.size() - 1)
        {
            polyline.erase(polyline.begin() + endIdx + 1, polyline.end());
        }
        if (startIdx > 0)
        {
            polyline.erase(polyline.begin(), polyline.begin() + startIdx);
        }
    }
}

/*!
 * @brief 歩道中心線作成
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CNetworkCreator::CreateFootpathCenterLines()
{
    for (auto tranRoadData : m_tranRoadData)
    {
        for (auto edgePair : tranRoadData->m_footpath.edgePairList)
        {
            // 中心線
            Boost3DHashPolyline centerLine;
            if (CreatePolylineBetweenTwoPolylines(edgePair, centerLine))
            {
                // 補間用データの準備
                Boost3DHashMultiPolygon targetPolygonList;
                switch (m_iLod)
                {
                case 2:
                    for (auto lod2 : tranRoadData->m_lod2List)
                    {
                        // 中心線と対応する歩道部のみ
                        if (lod2.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH)
                            && !CBoostGeoUtil::Disjoint(lod2.m_boostGeometry, centerLine))
                            targetPolygonList.emplace_back(lod2.m_boostGeometry);
                    }
                    break;
                case 3:
                    for (auto lod3 : tranRoadData->m_lod3List)
                    {
                        // 中心線と対応する歩道部のみ
                        if (lod3.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH)
                            && !CBoostGeoUtil::Disjoint(lod3.m_boostGeometry, centerLine))
                            targetPolygonList.emplace_back(lod3.m_boostGeometry);
                    }
                    break;
                default:
                    break;
                }

                // 不足分の補間
                Boost3DHashPolyline extendCenterLine = ExtendPolylineUntilPolygon(centerLine, targetPolygonList);
                CCenterLineData centerLineData(extendCenterLine);

                // tranRoadDataに中心線を保存
                tranRoadData->m_footpath.centerLineList.emplace_back(std::make_shared<CCenterLineData>(centerLineData));

                // はみ出し確認
                Boost3DHashMultiLines outsideLines = checkOutsideOfPolygon(extendCenterLine, targetPolygonList);
                for (const auto &line : outsideLines)
                {
                    Boost3DPointHash pt;
                    bg::centroid(line, pt);
                    CErrLogger::GetInstance()->WriteFootpathLog(FootpathErrType::OUTSIDE_OF_POLYGON, pt);
                }
            }
        }
    }
    return true;
}

/*!
 * @brief 歩道の交差点接続
*/
void CNetworkCreator::FootpathConnectionByCrossing()
{
    const double dParallelAngleDiff = 20.0;
    for (auto tranRoadData : m_tranRoadData)
    {
        // 交差点以外はskip
        if (tranRoadData->m_nInOut < 3)
            continue;

        // 注目歩道ポリゴン
        Boost3DHashMultiPolygon targetPolygonList;

        // 使用するLODのデータを準備
        switch (m_iLod)
        {
        case 2:
            for (auto lod2 : tranRoadData->m_lod2List)
            {
                // 歩道部のみ
                if (lod2.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH))
                    targetPolygonList.emplace_back(lod2.m_boostGeometry);
            }
            break;
        case 3:
            targetPolygonList = CTranRoadDataUtil::GetFootpathAndPlantsPolygon(*tranRoadData);
            break;
        default:
            break;
        }

        // 注目交差点に歩道部ポリゴンがない場合はskip
        if (targetPolygonList.size() == 0)
            continue;

        // 注目歩道ごとに処理
        for (const auto &target : targetPolygonList)
        {
            // 近傍探索用データの準備
            Boost3DMultiPointHashs pts;
            for (size_t t = 0; t < target.outer().size() - 1; t++)
                pts.push_back(target.outer().at(t));
            CNearestNeighborSearch nn(pts);

            // 注目歩道部の中心線が存在するか確認する
            std::shared_ptr<CCenterLineData> targetCenterLinePtr = nullptr;
            for (const auto &centerLineDataPtr : tranRoadData->m_footpath.centerLineList)
            {
                if (!CBoostGeoUtil::Disjoint(target, centerLineDataPtr->centerLine))
                {
                    targetCenterLinePtr = centerLineDataPtr; // 中心線が存在する
                    break;
                }
            }

            // 注目歩道に隣接する歩道の探索
            std::vector<std::pair<BoostPolygon, std::shared_ptr<CCenterLineData>>> candidates;
            for (auto &neighborPtr : tranRoadData->m_neighborRoadPtr)
            {
                // 隣接歩道ポリゴン
                Boost3DHashMultiPolygon neighborPolygonList;
                switch (m_iLod)
                {
                case 2:
                    for (auto lod2 : neighborPtr->m_lod2List)
                    {
                        // 歩道部のみ
                        if (lod2.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH))
                            neighborPolygonList.emplace_back(lod2.m_boostGeometry);
                    }
                    break;
                case 3:
                    for (auto lod3 : neighborPtr->m_lod3List)
                    {
                        // 歩道部のみ
                        if (lod3.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH))
                            neighborPolygonList.emplace_back(lod3.m_boostGeometry);
                    }
                    break;
                default:
                    break;
                }

                for (const auto &neighbor : neighborPolygonList)
                {
                    bool bNeighbor = false;
                    for (size_t t = 0; t < neighbor.outer().size() - 1; t++)
                    {
                        Boost3DPointHash pt = neighbor.outer().at(t);
                        Boost3DMultiPointHashs knnPts = nn.NNSearch(pt, 1);
                        if (pt.IsRoundEqual(knnPts[0]))
                        {
                            bNeighbor = true;
                            break;
                        }
                    }

                    if (bNeighbor)
                    {
                        // 隣接歩道の中心線を探索
                        std::shared_ptr<CCenterLineData> neighborCenterLinePtr = nullptr;
                        for (auto &centerLineDataPtr : neighborPtr->m_footpath.centerLineList)
                        {
                            // 隣接歩道と中心線の衝突判定
                            if (!CBoostGeoUtil::Disjoint(neighbor, centerLineDataPtr->centerLine))
                            {
                                neighborCenterLinePtr = centerLineDataPtr;
                                break;
                            }
                        }
                        if (neighborCenterLinePtr != nullptr)
                        {
                            BoostPolygon tmpNeighbor = CBoostGeoUtil::Conv(neighbor);
                            candidates.push_back(
                                std::pair<BoostPolygon, std::shared_ptr<CCenterLineData>>(
                                    tmpNeighbor, neighborCenterLinePtr));
                        }
                    }
                }
            }

            if (targetCenterLinePtr == nullptr)
            {
                // 注目歩道部の中心線がない場合
                // 注目歩道の重心
                Boost3DPointHash centerPt;
                bg::centroid(target, centerPt);

                // 隣接歩道の中心線の始終点において交差点に近い点を取得する
                Boost3DMultiPointHashs startPts;
                std::vector<CVector2D> directions;
                for (const auto &candidate : candidates)
                {
                    double dDist1 = bg::distance(target, candidate.second->centerLine.front());
                    double dDist2 = bg::distance(target, candidate.second->centerLine.back());

                    CVector2D firstPt(candidate.second->centerLine.front().x(), candidate.second->centerLine.front().y());                // 始点
                    CVector2D nextPt((candidate.second->centerLine.begin() + 1)->x(), (candidate.second->centerLine.begin() + 1)->y());   // 始点の次点
                    CVector2D lastPt(candidate.second->centerLine.back().x(), candidate.second->centerLine.back().y());                   // 終点
                    CVector2D prevPt((candidate.second->centerLine.end() - 2)->x(), (candidate.second->centerLine.end() - 2)->y());       // 終点の前点
                    CVector2D vec1 = firstPt - nextPt;
                    CVector2D vec2 = lastPt - prevPt;
                    // 交差点に近い方の端点の座標と方向を保持する
                    Boost3DPointHash pt = CEpsUtil::Less(dDist1, dDist2) ? candidate.second->centerLine.front() : candidate.second->centerLine.back();
                    startPts.emplace_back(pt);
                    CVector2D vec = CEpsUtil::Less(dDist1, dDist2) ? vec1 : vec2;
                    directions.emplace_back(vec);
                }

                if (candidates.size() == 1)
                {
                    // 隣接歩道の中心線を延長する
                    CVector2D vecE = directions.front();
                    vecE.Normalize();
                    CVector2D startPos(startPts.front().x(), startPts.front().y());
                    CVector2D pos = vecE * 0.1 + startPos;
                    BoostPoint tmp2dPos(pos.x, pos.y);
                    BoostPolygon tmpPoly = CBoostGeoUtil::Conv(target);
                    if (bg::covered_by(tmp2dPos, tmpPoly))
                    {
                        Boost3DPointHash tmpPos(pos.x, pos.y, 0);
                        Boost3DHashPolyline centerLine;
                        centerLine.emplace_back(startPts.front());
                        centerLine.emplace_back(tmpPos);
                        Boost3DHashMultiPolygon tmpPolygons;
                        tmpPolygons.emplace_back(target);
                        Boost3DHashPolyline extendLine = ExtendPolylineUntilPolygon(centerLine, tmpPolygons);
                        if (extendLine.size() > 2)
                            extendLine.erase(extendLine.begin() + 1); // 次点(開始地点から10cm離れた点は不要なため削除
                        if (CEpsUtil::GreaterEqual(bg::length(extendLine), 1.0))
                        {
                            CCenterLineData centerLineData(extendLine);
                            tranRoadData->m_footpath.centerLineList.emplace_back(std::make_shared<CCenterLineData>(centerLineData));
                        }
                    }
                }
                else if (candidates.size() == 2)
                {
                    CVector2D startPt(startPts.front().x(), startPts.front().y());
                    CVector2D endPt(startPts.back().x(), startPts.back().y());
                    CVector2D middlePt(centerPt.x(), centerPt.y());

                    // 交点確認
                    bool bOnline1, bOnline2;
                    double s, t;
                    if (CGeoUtil::GetCrossPos(
                        directions.front(), startPt, directions.back(), endPt,
                        middlePt, bOnline1, bOnline2, t, s))
                    {
                        BoostPoint tmpPt(middlePt.x, middlePt.y);
                        BoostPolygon tmpPoly = CBoostGeoUtil::Conv(target);
                        if (bg::covered_by(tmpPt, tmpPoly))
                        {
                            // 中間点を重心から交点に変更
                            centerPt.x(middlePt.x);
                            centerPt.y(middlePt.y);
                        }
                    }

                    // 始終点を繋いだ線分と各隣接歩道部の中心線の線分との角度が水平に近い場合は直繋ぎ
                    Boost3DHashPolyline centerLine;
                    centerLine.emplace_back(startPts.front());
                    centerLine.emplace_back(startPts.back());
                    CVector2D vec = endPt - startPt;
                    double dAngle1 = CGeoUtil::Angle(vec, directions.front());
                    double dAngle2 = CGeoUtil::Angle(vec, directions.back());
                    if (CEpsUtil::Greater(dAngle1, 90.0))
                        dAngle1 = 180 - dAngle1;
                    if (CEpsUtil::Greater(dAngle2, 90.0))
                        dAngle2 = 180 - dAngle2;
                    if (CEpsUtil::Greater(dAngle1, dParallelAngleDiff)
                        || CEpsUtil::Greater(dAngle2, dParallelAngleDiff))
                    {
                        // 直繋ぎでは角度が付く場合は中間点を挿入する
                        centerLine.insert(centerLine.begin() + 1, centerPt);
                    }
                    CCenterLineData centerLineData(centerLine);
                    tranRoadData->m_footpath.centerLineList.emplace_back(std::make_shared<CCenterLineData>(centerLineData));
                }
                else if (candidates.size() > 2)
                {
                    // 隣接歩道部が3以上は中間点で接続
                    BoostPolygon tmpPoly = CBoostGeoUtil::Conv(target);
                    Boost3DPointHashCntMap crossPtMap;
                    for (size_t i = 0; i < startPts.size() - 1; i++)
                    {
                        for (size_t j = i + 1; j < startPts.size(); j++)
                        {
                            double dAngle = CGeoUtil::Angle(directions[i], directions[j]);
                            if (CEpsUtil::Greater(dAngle, 90.0))
                                dAngle = 180 - dAngle;
                            if (CEpsUtil::LessEqual(dAngle, 3.0))
                                continue;   // 平行な場合はskip

                            CVector2D pt1(startPts[i].x(), startPts[i].y());
                            CVector2D pt2(startPts[j].x(), startPts[j].y());

                            // 交点確認
                            CVector2D crossPt;
                            bool bOnline1, bOnline2;
                            double s, t;
                            if (CGeoUtil::GetCrossPos(
                                directions[i], pt1, directions[j], pt2,
                                crossPt, bOnline1, bOnline2, t, s))
                            {
                                BoostPoint tmpPt(crossPt.x, crossPt.y);
                                if (bg::covered_by(tmpPt, tmpPoly))
                                {
                                    Boost3DPointHash insertPt(crossPt.x, crossPt.y, 0);
                                    auto it = crossPtMap.begin();
                                    for (; it != crossPtMap.end(); it++)
                                        if (it->first.IsRoundEqual(insertPt))
                                            break;
                                    if (it != crossPtMap.end())
                                        it->second += 1;
                                    else
                                        crossPtMap.insert(Boost3DPointHashCntMap::value_type(insertPt, 1));
                                }
                            }
                        }
                    }

                    if (crossPtMap.size() > 0)
                    {
                        int nCnt = 0;
                        for (const auto &val : crossPtMap)
                        {
                            if (nCnt < val.second)
                            {
                                nCnt = val.second;
                                centerPt = val.first;   // 最頻点に中点を変更
                            }
                        }
                    }

                    for (const auto &candidate : candidates)
                    {
                        // 隣接歩道部の中心線の端点と算出した交点を結ぶ
                        double dDist1 = centerPt.RoundDistance(candidate.second->centerLine.front());
                        double dDist2 = centerPt.RoundDistance(candidate.second->centerLine.back());
                        Boost3DPointHash pt = CEpsUtil::Less(dDist1, dDist2) ? candidate.second->centerLine.front() : candidate.second->centerLine.back();

                        Boost3DHashPolyline centerLine;
                        centerLine.emplace_back(pt);
                        centerLine.emplace_back(centerPt);
                        CCenterLineData centarLineData(centerLine);
                        tranRoadData->m_footpath.centerLineList.emplace_back(std::make_shared<CCenterLineData>(centarLineData));
                    }
                }
            }
            else if (candidates.size() > 0)
            {
                // 注目歩道部の中心線が存在し、隣接歩道部の中心線も存在する場合
                Boost3DMultiPointHashs startEndPts;
                for (const auto &candidate : candidates)
                {
                    startEndPts.emplace_back(candidate.second->centerLine.front());
                    startEndPts.emplace_back(candidate.second->centerLine.back());
                }

                // 近傍点探索
                CNearestNeighborSearch centerLineNn(startEndPts);
                Boost3DMultiPointHashs  frontKnnPts = centerLineNn.NNSearch(targetCenterLinePtr->centerLine.front(), 1);
                Boost3DMultiPointHashs backKnnPts = centerLineNn.NNSearch(targetCenterLinePtr->centerLine.back(), 1);
                CVector2D vecFront(frontKnnPts[0].x() - targetCenterLinePtr->centerLine.front().x(),
                    frontKnnPts[0].y() - targetCenterLinePtr->centerLine.front().y());
                CVector2D vecBack(backKnnPts[0].x() - targetCenterLinePtr->centerLine.back().x(),
                    backKnnPts[0].y() - targetCenterLinePtr->centerLine.back().y());

                // ループ用の準備
                std::vector<Boost3DPointHash> nnPts;
                nnPts.emplace_back(frontKnnPts[0]);
                nnPts.emplace_back(backKnnPts[0]);
                std::vector<CVector2D> directions;
                directions.emplace_back(vecFront);
                directions.emplace_back(vecBack);
                std::vector<std::pair<Boost3DHashPolyline::iterator, Boost3DHashPolyline::iterator>> checkPts;
                checkPts.emplace_back(std::pair<Boost3DHashPolyline::iterator, Boost3DHashPolyline::iterator>(
                    targetCenterLinePtr->centerLine.begin(), targetCenterLinePtr->centerLine.begin() + 1));
                checkPts.emplace_back(std::pair<Boost3DHashPolyline::iterator, Boost3DHashPolyline::iterator>(
                    targetCenterLinePtr->centerLine.end() - 1, targetCenterLinePtr->centerLine.end() - 2));
                bool bFrontIsShortLength = CEpsUtil::Less(vecFront.Length(), vecBack.Length());

                // 頂点編集確認
                for (size_t i = 0; i < checkPts.size(); i++)
                {
                    // 隣接歩道の中心線が1本の場合、始終点どちらを編集するか決定するフラグ
                    bool bNeighborFlag;
                    if (i == 0)
                        bNeighborFlag = (candidates.size() == 1 && bFrontIsShortLength);
                    else
                        bNeighborFlag = (candidates.size() == 1 && !bFrontIsShortLength);

                    if ((candidates.size() > 1 || bNeighborFlag)
                        && !checkPts[i].first->IsRoundEqual(nnPts[i]))
                    {
                        CVector2D vec(checkPts[i].first->x() - checkPts[i].second->x(),
                            checkPts[i].first->y() - checkPts[i].second->y());
                        double dAngle = CGeoUtil::Angle(directions[i], vec);
                        if (CEpsUtil::Less(dAngle, 150.0)   // 注目歩道内に折り返さないか確認
                            && CEpsUtil::Greater(directions[i].Length(), 0))
                        {
                            CVector3D pt1(checkPts[i].first->x(), checkPts[i].first->y(), checkPts[i].first->z());
                            CVector3D pt2(nnPts[i].x(), nnPts[i].y(), nnPts[i].z());
                            CVector3D vec = pt2 - pt1;
                            CVector3D middlePt = vec * 0.5 + pt1;

                            checkPts[i].first->x(middlePt.x);
                            checkPts[i].first->y(middlePt.y);
                            checkPts[i].first->z(middlePt.z);

                            for (auto &candidate : candidates)
                            {
                                if (candidate.second->centerLine.begin()->IsRoundEqual(nnPts[i]))
                                {
                                    candidate.second->centerLine.begin()->x(middlePt.x);
                                    candidate.second->centerLine.begin()->y(middlePt.y);
                                    candidate.second->centerLine.begin()->z(middlePt.z);
                                    break;
                                }
                                if ((candidate.second->centerLine.end() - 1)->IsRoundEqual(nnPts[i]))
                                {
                                    (candidate.second->centerLine.end() - 1)->x(middlePt.x);
                                    (candidate.second->centerLine.end() - 1)->y(middlePt.y);
                                    (candidate.second->centerLine.end() - 1)->z(middlePt.z);
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

/*!
 * @brief 横断歩道による歩道の接続
 * @param[in] dSamplingInterval 歩道中心線のサンプリング間隔m
 * @param[in] dNNFootpathDistTh 最近傍歩道を探索する際の距離しきい値距離閾値m
 * @param[in] dNNPCDistTh       横断歩道同士を接続する際の探索範囲m
*/
void CNetworkCreator::FootpathConnectionByPedestrianCrossing(
    const double dSamplingInterval,
    const double dNNFootpathDistTh,
    const double dNNPCDistTh,
    const double dSearchPCDistTh)
{
    // コンセプト
    // 近傍歩道中心線と接続する
    // 近傍歩道中心線と接続しなかった部分は近傍横断歩道と接続する

    // 重畳道路探索用データの作成
    CSearchOverlapRoads sor;
    sor.SetData(m_tranRoadData, CInputSettingData::GetInstance()->lodType);

    // 重複横断歩道の探索
    // 歩行者横断歩道と自転車横断歩道が隣接している場合は自転車横断歩道を使用しない設定に変更する
    CFurnitureDataUtil::DuplicateCheck(m_pedestrianCrossingData);

    // 近傍横断歩道探索用RTree
    SearchPCCenterLineRTree spcclRtree;
    for (auto &crossing : m_pedestrianCrossingData)
    {
        if (crossing->m_pedestrianCrossingData.m_bUse)
        {
            spcclRtree.insert({ crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.front(), crossing, true });
            spcclRtree.insert({ crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.back(), crossing, false });
        }
    }

    // 横断歩道を近傍歩道の中心線または近傍横断歩道と接続する
    // 近傍歩道は、横断歩道のMBRと重複する道路と重複道路と隣接する道路から探索する
    for (auto &crossing : m_pedestrianCrossingData)
    {
        if (crossing->m_pedestrianCrossingData.m_bUse)
        {
            // 重畳道路の探索
            auto map = sor.Search(crossing->m_pedestrianCrossingData.m_mbr);
            if (map.size() == 0)
            {
                // 重畳道路がない場合は接続対象外の横断歩道とする
                crossing->m_pedestrianCrossingData.m_bUse = false;
                continue;
            }

            // 歩道と接続するか横断歩道と接続するか決定する用
            double dNNFrontDist = DBL_MAX;  // 最近傍歩道ポリゴンと始点との距離
            double dNNBackDist = DBL_MAX;   // 最近傍歩道ポリゴンと終点との距離
            std::shared_ptr<CTranRoadData> nnFrontRoadPtr = nullptr;
            std::shared_ptr<CTranRoadData> nnBackRoadPtr = nullptr;

            // 横断歩道の始終点
            Boost3DPointHash startPt = crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.front();
            Boost3DPointHash endPt = crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.back();

            // 横断歩道の中心線ベクトル
            CVector2D pt1 = CBoostGeoUtil::ToCVector2D(startPt);
            CVector2D pt2 = CBoostGeoUtil::ToCVector2D(endPt);
            CVector2D vec = pt1 - pt2;

            SearchCenterLineRTree sclRtree;                         // 歩道中心線探索用RTree
            std::set<std::shared_ptr<CTranRoadData>> searchedRoads; // 探索済み道路確認用
            for (const auto &data : map)
            {
                if (searchedRoads.find(data.first) == searchedRoads.end())
                {
                    // 未探索の場合
                    // 最近傍歩道の探索
                    // 歩道と接続するか横断歩道と接続するか決定するために
                    // 最近傍歩道を含む道路ポインタと歩道までの距離を取得する
                    searchNNFootpathPolygon(
                        data.first, vec, pt2, dNNFrontDist, nnFrontRoadPtr, dNNBackDist, nnBackRoadPtr);

                    // 探索結果の歩道中心線を登録
                    updateFootpathCenterLineRTree(sclRtree, data.first, dSamplingInterval);
                    searchedRoads.insert(data.first);
                }

                // 隣接道路の歩道中心線も登録
                for (auto &neighborRoad : data.first->m_neighborRoadPtr)
                {
                    if (searchedRoads.find(neighborRoad) == searchedRoads.end())
                    {
                        // 最近傍歩道の探索
                        searchNNFootpathPolygon(
                            data.first, vec, pt2, dNNFrontDist, nnFrontRoadPtr, dNNBackDist, nnBackRoadPtr);

                        // 探索結果の歩道中心線を登録
                        updateFootpathCenterLineRTree(sclRtree, neighborRoad, dSamplingInterval);
                        searchedRoads.insert(neighborRoad);
                    }
                }
            }

            // searchNNFootpathPolygonにて、横断歩道の中心線が歩道ポリゴンと交差する場合は
            // 交差地点で横断歩道の中心線をカットしているため、横断歩道の中心線の始終点を更新する
            // 横断歩道の中心線の向きvecと基点pt2はsearchNNFootpathPolygonで更新済み
            pt1 = vec + pt2;
            startPt.x(pt1.x);
            startPt.y(pt1.y);
            endPt.x(pt2.x);
            endPt.y(pt2.y);

            // 近傍歩道中心線の探索
            std::vector<CenterLineTuple> nnResult1, nnResult2;
            sclRtree.query(bg::index::nearest(startPt, 1)
                && bg::index::satisfies([startPt, vec, nnFrontRoadPtr, dNNFootpathDistTh](const CenterLineTuple &v)
                    {
                        auto [pt, pRoad, itCenterLine, itPt, bFlag] = v;
                        CVector2D tmpPt1 = CBoostGeoUtil::ToCVector2D(startPt);
                        CVector2D tmpPt2 = CBoostGeoUtil::ToCVector2D(pt);
                        double dIP = CGeoUtil::InnerProduct(vec, tmpPt2 - tmpPt1);
                        // 近傍歩道の中心線の点、または、近傍歩道の中心点ではないが距離がしきい値未満
                        // かつ、横断歩道の終点->始点方向側に存在する近傍歩道の中心点である
                        return (pRoad == nnFrontRoadPtr
                            || (pRoad != nnFrontRoadPtr
                                && CEpsUtil::Less(bg::distance(pt, startPt), dNNFootpathDistTh)))
                            && CEpsUtil::Greater(dIP, 0);
                    }), std::back_inserter(nnResult1));
            sclRtree.query(bg::index::nearest(endPt, 1)
                && bg::index::satisfies([endPt, vec, nnBackRoadPtr, dNNFootpathDistTh](const CenterLineTuple &v)
                    {
                        auto [pt, pRoad, itCenterLine, itPt, bFlag] = v;
                        CVector2D tmpPt1 = CBoostGeoUtil::ToCVector2D(endPt);
                        CVector2D tmpPt2 = CBoostGeoUtil::ToCVector2D(pt);
                        double dIP = CGeoUtil::InnerProduct(vec * -1, tmpPt2 - tmpPt1);
                        // 近傍歩道の中心線の点、または、近傍歩道の中心点ではないが距離がしきい値未満
                        // かつ、横断歩道の始点->終点方向側に存在する近傍歩道の中心点である
                        return (pRoad == nnBackRoadPtr
                                || (pRoad != nnBackRoadPtr
                                    && CEpsUtil::Less(bg::distance(pt, endPt), dNNFootpathDistTh)))
                                && CEpsUtil::Greater(dIP, 0);
                    }), std::back_inserter(nnResult2));

            // 近傍横断歩道の中心線探索
            std::vector<PCCenterLineTuple> nnpcResult1, nnpcResult2;
            spcclRtree.query(bg::index::nearest(startPt, 1)
                && bg::index::satisfies([crossing, startPt, vec, dNNPCDistTh](const PCCenterLineTuple &v)
                    {
                        const auto [pt, pCross, bFront] = v;
                        CVector2D tmpPt1 = CBoostGeoUtil::ToCVector2D(startPt);
                        CVector2D tmpPt2 = CBoostGeoUtil::ToCVector2D(pt);
                        double dIP = CGeoUtil::InnerProduct(vec, tmpPt2 - tmpPt1);
                        // 注目横断歩道ではない、かつ、横断歩道の始点と近傍横断歩道の端点間の距離がしきい値未満
                        // かつ、横断歩道の終点->始点方向側に存在する近傍横断歩道の端点
                        return pCross != crossing && CEpsUtil::Less(bg::distance(pt, startPt), dNNPCDistTh) && CEpsUtil::Greater(dIP, 0);
                    }), std::back_inserter(nnpcResult1));
            spcclRtree.query(bg::index::nearest(endPt, 1)
                && bg::index::satisfies([crossing, endPt, vec, dNNPCDistTh](const PCCenterLineTuple &v)
                    {
                        const auto [pt, pCross, bFront] = v;
                        CVector2D tmpPt1 = CBoostGeoUtil::ToCVector2D(endPt);
                        CVector2D tmpPt2 = CBoostGeoUtil::ToCVector2D(pt);
                        double dIP = CGeoUtil::InnerProduct(vec * -1, tmpPt2 - tmpPt1);
                        // 注目横断歩道ではない、かつ、横断歩道の終点と近傍横断歩道の端点間の距離がしきい値未満
                        // かつ、横断歩道の始点->終点方向側に存在する近傍横断歩道の端点
                        return pCross != crossing && CEpsUtil::Less(bg::distance(pt, endPt), dNNPCDistTh) && CEpsUtil::Greater(dIP, 0);
                    }), std::back_inserter(nnpcResult2));

            // 始点側の歩道中心線との接続確認
            bool bUpdateStart = false;
            if (nnResult1.size() > 0)
            {
                // 近傍道路中心線
                auto [pt, pRoad, itCenterLine, itPt, bFlag] = nnResult1[0];
                double dFrontDist1 = bg::distance(startPt, pt);    // 横断歩道の始点から近傍中心線までの距離
                if (CEpsUtil::Less(dFrontDist1, dNNFrontDist))
                    dNNFrontDist = dFrontDist1; // 近傍歩道中心線までの距離が近傍歩道ポリゴンまでの距離より短い場合

                bool bNearlyPCFront = false;
                if (nnpcResult1.size() > 0)
                {
                    // 近傍横断歩道
                    auto [pcPt, pCrossing, bFront] = nnpcResult1[0];
                    double dFrontDist2 = bg::distance(startPt, pcPt);  // 横断歩道の始点から近傍横断歩道までの距離
                    bNearlyPCFront = CEpsUtil::Less(dFrontDist2, dNNFrontDist);
                }

                if (!bNearlyPCFront)
                {
                    bUpdateStart = true;    // 近傍横断歩道より近傍歩道が近いため歩道中心線と接続する
                }
            }
            // 終点側の歩道中心線との接続確認
            bool bUpdateEnd = false;
            if (nnResult2.size() > 0)
            {
                // 近傍道路中心線
                auto [pt, pRoad, itCenterLine, itPt, bFlag] = nnResult2[0];
                double dBackDist1 = bg::distance(endPt, pt);    // 横断歩道の始点から近傍中心線までの距離
                if (CEpsUtil::Less(dBackDist1, dNNBackDist))
                    dNNBackDist = dBackDist1;    // 近傍歩道中心線までの距離が近傍歩道ポリゴンまでの距離より短い場合

                bool bNearlyPCBack = false;
                if (nnpcResult2.size() > 0)
                {
                    // 近傍横断歩道
                    auto [pcPt, pCrossing, bFront] = nnpcResult2[0];
                    double dBackDist2 = bg::distance(endPt, pcPt);  // 横断歩道の終点から近傍横断歩道までの距離
                    bNearlyPCBack = CEpsUtil::Less(dBackDist2, dNNBackDist);
                }

                if (!bNearlyPCBack)
                {
                    bUpdateEnd = true;  // 近傍横断歩道より近傍歩道が近いため歩道中心線と接続する
                }
            }

            if (bUpdateStart && bUpdateEnd)
            {
                // 横断歩道の始終点ともに近傍歩道中心線と接続する場合
                auto [pt1, pRoad1, itCenterLine1, itPt1, bFlag1] = nnResult1[0];
                auto [pt2, pRoad2, itCenterLine2, itPt2, bFlag2] = nnResult2[0];
                if (itCenterLine1 == itCenterLine2)
                {
                    // 接続対象の近傍歩道中心線が同一の場合は接続しない
                    crossing->m_pedestrianCrossingData.m_bUse = false;
                    continue;
                }
            }

            if (bUpdateStart)
            {
                // 横断歩道の始点と近傍歩道中心線の接続
                auto [pt, pRoad, centerLineDataPtr, itPt, bFlag] = nnResult1[0];
                crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.begin()->x(pt.x());
                crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.begin()->y(pt.y());
                crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.begin()->z(pt.z());
                crossing->m_pedestrianCrossingData.m_bFrontConnection = true;   // 接続済みに変更

                if (bFlag)
                {
                    // サンプリング点の場合は近傍歩道中心線に点を追加する
                    centerLineDataPtr->centerLine.insert(itPt + 1, pt);
                }
            }

            if (bUpdateEnd)
            {
                // 横断歩道の終点と近傍歩道中心線の接続
                auto [pt, pRoad, centerLineDataPtr, itPt, bFlag] = nnResult2[0];
                (crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.end() - 1)->x(pt.x());
                (crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.end() - 1)->y(pt.y());
                (crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.end() - 1)->z(pt.z());
                crossing->m_pedestrianCrossingData.m_bBackConnection = true;    // 接続済みに変更

                if (bFlag)
                {
                    // サンプリング点の場合は近傍歩道中心線に点を追加する
                    centerLineDataPtr->centerLine.insert(itPt + 1, pt);
                }
            }
        }
    }

    // 歩道中心線と未接続な横断歩道を対象に横断歩道同士での接続を確認する
    for (auto &crossing : m_pedestrianCrossingData)
    {
        if (crossing->m_pedestrianCrossingData.m_bUse)
        {
            std::vector<std::pair<Boost3DPointHash, bool>> data;
            data.emplace_back(std::make_pair(crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.front(), true));
            data.emplace_back(std::make_pair(crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.back(), false));

            for (const auto &datum : data)
            {
                // 接続状況
                bool bTargetConnection = datum.second ?
                    crossing->m_pedestrianCrossingData.m_bFrontConnection
                    : crossing->m_pedestrianCrossingData.m_bBackConnection;
                if (!bTargetConnection)
                {
                    // 未接続の場合
                    Boost3DPointHash targetPt = datum.first;
                    std::vector<PCCenterLineTuple> nnpcResult;
                    spcclRtree.query(bg::index::satisfies([targetPt, dSearchPCDistTh](const PCCenterLineTuple &v)
                        {
                            const auto [pt, pCross, bFront] = v;
                            bool bConnection = bFront ? pCross->m_pedestrianCrossingData.m_bFrontConnection : pCross->m_pedestrianCrossingData.m_bBackConnection;
                            // 注目横断歩道の端点から近傍横断歩道の端点間の距離がしきい値以下
                            // かつ、近傍横断歩道が接続対象
                            // かつ、近傍横断歩道が未接続の端点を持つ
                            return (CEpsUtil::LessEqual(pt.RoundDistance(targetPt), dSearchPCDistTh)
                                && pCross->m_pedestrianCrossingData.m_bUse
                                && !bConnection);
                        }), std::back_inserter(nnpcResult));
                    if (nnpcResult.size() > 1)
                    {
                        // 同一横断歩道において両端の端点を取得している場合は距離が近い方を採用する
                        std::map<std::shared_ptr<CFurnitureData>, PCCenterLineTuple> tmpMap;
                        for (const auto &result : nnpcResult)
                        {
                            const auto [pt, pCross, bFront] = result;
                            auto it = tmpMap.find(pCross);
                            if (it == tmpMap.end())
                            {
                                tmpMap.insert(std::make_pair(pCross, result));
                            }
                            else
                            {
                                double dCandidate = pt.RoundDistance(targetPt);
                                const auto [tmpPt, pTmpCross, bTmpFront] = it->second;
                                double dCurrent = tmpPt.RoundDistance(targetPt);
                                if (CEpsUtil::Less(dCandidate, dCurrent))
                                {
                                    it->second = result;  // 距離が近い方に差し替える
                                }
                            }
                        }

                        // 探索結果には注目横断歩道も含まれるため結果が2個以上の場合は接続する
                        Boost3DMultiPointHashs pts;
                        for (const auto &val : tmpMap)
                        {
                            const auto [pt, pCross, bFront] = val.second;
                            pts.emplace_back(pt);
                        }
                        if (pts.size() > 1)
                        {
                            Boost3DPointHash centerPt;
                            bg::centroid(pts, centerPt);
                            for (const auto &val : tmpMap)
                            {
                                const auto [pt, pCross, bFront] = val.second;
                                if (bFront)
                                {
                                    pCross->m_pedestrianCrossingData.m_centerLineData.centerLine.begin()->x(centerPt.x());
                                    pCross->m_pedestrianCrossingData.m_centerLineData.centerLine.begin()->y(centerPt.y());
                                    pCross->m_pedestrianCrossingData.m_centerLineData.centerLine.begin()->z(centerPt.z());
                                    pCross->m_pedestrianCrossingData.m_bFrontConnection = true;
                                }
                                else
                                {
                                    (pCross->m_pedestrianCrossingData.m_centerLineData.centerLine.end() - 1)->x(centerPt.x());
                                    (pCross->m_pedestrianCrossingData.m_centerLineData.centerLine.end() - 1)->y(centerPt.y());
                                    (pCross->m_pedestrianCrossingData.m_centerLineData.centerLine.end() - 1)->z(centerPt.z());
                                    pCross->m_pedestrianCrossingData.m_bBackConnection = true;
                                }
                            }
                        }
                    }

                    // 接続状況の更新
                    bTargetConnection = datum.second ?
                        crossing->m_pedestrianCrossingData.m_bFrontConnection
                        : crossing->m_pedestrianCrossingData.m_bBackConnection;
                }

                // 接続エラーチェック
                if (!bTargetConnection)
                {
                    Boost3DPointHash pt = datum.second ?
                        crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.front()
                        : crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.back();
                    CErrLogger::GetInstance()->WriteFootpathLog(
                        FootpathErrType::DISCONNECTED_PEDESTRIAN_CROSSING, pt);
                }
            }
        }
    }
}

/*!
 * @brief 最近傍歩道ポリゴンの探索
 * @param[in] roadPtr           注目道路ポインタ
 * @param[in/out] vec           最近傍探索対象のベクトル
 * @param[in/out] pt            ベクトルの始点
 * @param[in/out] dNNFrontDist  ベクトルの始点と最近傍歩道との距離
 * @param[out] nnFrontRoadPtr   ベクトルの始点と最近傍な歩道を含む道路のポインタ
 * @param[in/out] dNNBackDist   ベクトルの終点と最近傍歩道との距離
 * @param[out] nnBackRoadPtr    ベクトルの終点と最近傍な歩道を含む道路のポインタ
 * @return  処理結果
 * @retval  true    近傍歩道との距離と道路ポインタを更新した
 * @retval  false   近傍歩道との距離と道路ポインタを更新していない
*/
bool CNetworkCreator::searchNNFootpathPolygon(
    const std::shared_ptr<CTranRoadData> &roadPtr,
    CVector2D &vec,
    CVector2D &pt,
    double &dNNFrontDist,
    std::shared_ptr<CTranRoadData> &nnFrontRoadPtr,
    double &dNNBackDist,
    std::shared_ptr<CTranRoadData> &nnBackRoadPtr)
{
    bool bRet = false;

    // 入力ポリゴンの選別
    Boost3DHashMultiPolygon polys;
    if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD2)
    {
        for (const auto &lod : roadPtr->m_lod2List)
        {
            if (lod.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH))
            {
                for (const auto &centerLineData : roadPtr->m_footpath.centerLineList)
                {
                    if (!CBoostGeoUtil::Disjoint(lod.m_boostGeometry, centerLineData->centerLine))
                    {
                        // 中心線を保持する歩道ポリゴンのみ抽出
                        polys.emplace_back(lod.m_boostGeometry);
                        break;
                    }
                }
            }
        }
    }
    else if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
    {
        for (const auto &lod : roadPtr->m_lod3List)
        {
            if (lod.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH))
            {
                for (const auto &centerLineData : roadPtr->m_footpath.centerLineList)
                {
                    if (!CBoostGeoUtil::Disjoint(lod.m_boostGeometry, centerLineData->centerLine))
                    {
                        // 中心線を保持する歩道ポリゴンのみ抽出
                        polys.emplace_back(lod.m_boostGeometry);
                        break;
                    }
                }
            }
        }
    }

    // 横断歩道の始終点と最近傍歩道ポリゴンとの距離
    for (const auto &poly : polys)
    {
        for (auto it = poly.outer().cbegin(); it < poly.outer().cend() - 1; it++)
        {
            CVector2D tmpPt1 = CBoostGeoUtil::ToCVector2D(*it);
            CVector2D tmpPt2 = CBoostGeoUtil::ToCVector2D(*(it + 1));
            CVector2D tmpVec = tmpPt2 - tmpPt1;
            bool bOnline1, bOnline2;
            double t, s;
            CVector2D crossPt;
            if (CGeoUtil::GetCrossPos(tmpVec, tmpPt1, vec, pt, crossPt, bOnline1, bOnline2, t, s)
                && bOnline1)
            {
                if (CEpsUtil::Greater(s, 0.5))
                {
                    if (bOnline2)
                    {
                        // 横断歩道の中心線が歩道ポリゴンと交差する場合は交差地点でカットする
                        vec *= s;
                    }
                    double dFrontDist = (crossPt - (vec + pt)).Length();
                    if (CEpsUtil::Less(dFrontDist, dNNFrontDist))
                    {
                        dNNFrontDist = dFrontDist;
                        nnFrontRoadPtr = roadPtr;
                        bRet = true;
                    }
                }
                else
                {
                    if (bOnline2)
                    {
                        // 横断歩道の中心線が歩道ポリゴンと交差する場合は交差地点でカットする
                        pt = vec * s + pt;
                        vec *= (1.0 - s);
                    }
                    double dBackDist = (crossPt - pt).Length();
                    if (CEpsUtil::Less(dBackDist, dNNBackDist))
                    {
                        dNNBackDist = dBackDist;
                        nnBackRoadPtr = roadPtr;
                        bRet = true;
                    }
                }
            }
        }
    }

    return bRet;
}

/*!
 * @brief 近傍歩道中心線内の近傍点探索用RTreeの作成
 * @param[in] roadPtr           道路ポインタ
 * @param[in/out] rtree         rtree
 * @param[in] dSamplingInterval 中心線サンプリング間隔m
*/
void CNetworkCreator::updateFootpathCenterLineRTree(
    SearchCenterLineRTree &rtree,
    const std::shared_ptr<CTranRoadData> &roadPtr,
    const double dSamplingInterval)
{
    for (auto centerLineDataPtr : roadPtr->m_footpath.centerLineList)
    {
        for (auto itPt = centerLineDataPtr->centerLine.begin(); itPt < centerLineDataPtr->centerLine.end() - 1; itPt++)
        {
            // sampling
            CVector3D currPt = CBoostGeoUtil::ToCVector3D(*itPt);
            CVector3D nextPt = CBoostGeoUtil::ToCVector3D(*(itPt + 1));
            CVector3D vec = nextPt - currPt;
            double dMaxRate = vec.Length() / dSamplingInterval;
            vec.Normalize();
            for (double r = 0; CEpsUtil::Less(r, dMaxRate); r += dSamplingInterval)
            {
                bool bFlag = (CEpsUtil::Zero(r)) ? false : true;    // サンプリングで追加した点か否か
                CVector3D smpPt = vec * dSamplingInterval * r + currPt;
                rtree.insert({ Boost3DPointHash(smpPt.x, smpPt.y, smpPt.z), roadPtr, centerLineDataPtr, itPt, bFlag });
            }
        }
        rtree.insert({ centerLineDataPtr->centerLine.back(), roadPtr, centerLineDataPtr, centerLineDataPtr->centerLine.end() - 1, false });
    }
}

/*!
 * @brief 横断歩道橋の中心線作成
 * @param[in] dInterval サンプリング間隔m(幅員計測用)
*/
void CNetworkCreator::CreateCenterLineOfPedestrianBridge(const double dInterval)
{
    for (auto &bridge : m_bridgeData)
    {
        // 横断歩道橋読み込み時にLOD2は融解済み
        // 融解結果の理想は橋梁の外輪郭線のポリゴンが1つ
        if (bridge->m_lod2List.size() == 1)
        {
            // 細線化処理に横断歩道橋の中心線の作成
            std::vector<std::tuple<Boost3DHashPolyline, bool, bool>> tmpCenterLines = COpenCVUtil::Thinning(
                bridge->m_lod2List.front().m_boostGeometry, 0.1, 0.3, 3.0);

            if (tmpCenterLines.size() > 0)
            {
                // 標高値設定準備
                CSearchOverlapBridge sob;
                std::vector<std::shared_ptr<CBridgeData>> srcBridges;
                srcBridges.emplace_back(bridge);
                sob.SetData(srcBridges);

                // 幅員計測準備
                std::vector<std::pair<CVector2D, CVector2D>> edgeVecPairList; // エッジの始点と方向ベクトルのペア
                for (auto it = bridge->m_lod2List.front().m_boostGeometry.outer().cbegin();
                    it < bridge->m_lod2List.front().m_boostGeometry.outer().cend() - 1; it++)
                {
                    CVector2D startPoint = CBoostGeoUtil::ToCVector2D(*it);
                    CVector2D endPoint = CBoostGeoUtil::ToCVector2D(*(it + 1));
                    CVector2D edgeVec = endPoint - startPoint;
                    edgeVecPairList.emplace_back(std::make_pair(edgeVec, startPoint));
                }

                for (auto &tmp : tmpCenterLines)
                {
                    auto [centerLine, bFrontLinkagePt, bBackLinkagePt] = tmp;
                    CBridgeCenterLineData bridCenterLine;
                    bridCenterLine.m_centerLine.centerLine = centerLine;
                    bridCenterLine.m_bFrontLinkagePt = bFrontLinkagePt;
                    bridCenterLine.m_bBackLinkagePt = bBackLinkagePt;

                    // 標高値設定
                    // TODO:一旦入力道路がLOD3の場合に横断歩道橋の標高設定を行う
                    if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
                    {
                        std::vector<size_t> indexies;   // 高さ未設定インデックス一覧
                        for (size_t i = 0; i < bridCenterLine.m_centerLine.centerLine.size(); i++)
                        {
                            auto resultMap = sob.Search(bridCenterLine.m_centerLine.centerLine[i]);
                            CVector3D pt = CBoostGeoUtil::ToCVector3D(bridCenterLine.m_centerLine.centerLine[i]);
                            std::set<double> height;
                            for (const auto &result : resultMap)
                            {
                                for (const auto &poly : result.second)
                                {
                                    Boost3DPointHash crossPt;
                                    if (CBoostGeoUtil::CalcVerticalCrossPt(
                                        *poly.polygonPtr, bridCenterLine.m_centerLine.centerLine[i], crossPt))
                                    {
                                        height.insert(crossPt.z());
                                    }
                                }
                            }

                            if (height.size() > 0)
                            {
                                // OuterFloorSurfaceとOuterCeilingSurfaceから高さを取得しているため高い方(OuterFloorSurface)の高さを取得する
                                bridCenterLine.m_centerLine.centerLine[i].z(*(height.rbegin()));
                            }
                            else
                            {
                                indexies.emplace_back(i);
                            }
                        }

                        if (indexies.size() > 0)
                        {
                            // TODO
                            // 標高値が設定できなかった場合
                        }
                    }

                    // 幅員計測
                    double dMinWidth = 0;
                    Boost3DPointHash minWidthPos;
                    auto sampledCenterLine = CBoostGeoUtil::Sampling(centerLine, dInterval);

                    // 中心線のセグメント毎に処理
                    for (auto it = sampledCenterLine.cbegin();
                        it < sampledCenterLine.cend() - 1; it++)
                    {
                        CVector2D firstPoint = CBoostGeoUtil::ToCVector2D(*it);
                        CVector2D secondPoint = CBoostGeoUtil::ToCVector2D(*(it + 1));

                        CVector2D inputVec = secondPoint - firstPoint;
                        CVector2D rightVec, leftVec;

                        // 注目点(firstPoint)から左右に伸びる垂線ベクトルを求める
                        if (CGeoUtil::GetVerticalVec(inputVec, rightVec) == false)
                        {
                            continue;
                        }
                        leftVec = -1 * rightVec;

                        ///
                        /// 左右の交点算出
                        ///
                        CVector2D rightCrossPoint, leftCrossPoint;
                        double rightCrossLength = LDBL_MAX;
                        double leftCrossLength = LDBL_MAX;
                        bool isRightCrossPoint = false;
                        bool isLeftCrossPoint = false;

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
                            minWidthPos.x(centerPoint.x);
                            minWidthPos.y(centerPoint.y);
                        }
                    }
                    if (CEpsUtil::Greater(dMinWidth, 0))
                    {
                        bridCenterLine.m_centerLine.dMinWidth = dMinWidth;
                        bridCenterLine.m_centerLine.minWidthPos = minWidthPos;
                    }

                    // 中心線を保存
                    bridge->m_pedestrianBridgeData.m_centerLines.emplace_back(bridCenterLine);
                }
            }
        }
    }
}

/*!
 * @brief 横断歩道橋による歩道の接続
 * @param[in] dSamplingInterval 歩道中心線のサンプリング間隔m
 * @param[in] dDistTh           横断歩道橋と歩道を繋ぐエッジの距離閾値m
*/
void CNetworkCreator::FootpathConnectionByPedestrianBridge(
    const double dSamplingInterval,
    const double dDistTh)
{
    // コンセプト
    // 近傍歩道中心線と接続する

    // 重畳道路探索用データの作成
    CSearchOverlapRoads sor;
    sor.SetData(m_tranRoadData, CInputSettingData::GetInstance()->lodType);

    // 横断歩道橋を近傍歩道の中心線と接続する
    // 近傍歩道は横断歩道橋と重複する道路から探索する
    for (auto &bridge : m_bridgeData)
    {
        // 横断歩道橋読み込み時にLOD2は融解済み
        // 融解結果の理想は橋梁の外輪郭線のポリゴンが1つ
        if (bridge->m_lod2List.size() == 1)
        {
            // 重畳道路の探索
            auto map = sor.Search(bridge->m_lod2List.front().m_boostGeometry);
            if (map.size() == 0)
            {
                // 重畳道路がない場合は接続対象外の横断歩道橋とする
                bridge->m_pedestrianBridgeData.m_bUse = false;
                continue;
            }

            // 横断歩道橋の中心線ごとに処理
            for (auto &centerLine : bridge->m_pedestrianBridgeData.m_centerLines)
            {
                std::vector<std::tuple<Boost3DPointHash, bool>> targets;    // 接続対象点座標, 中心線の始点か否か
                if (!centerLine.m_bFrontLinkagePt)
                    targets.emplace_back(std::make_tuple(centerLine.m_centerLine.centerLine.front(), true));
                if (!centerLine.m_bBackLinkagePt)
                    targets.emplace_back(std::make_tuple(centerLine.m_centerLine.centerLine.back(), false));

                for (const auto &v : targets)
                {
                    bool bAddFlag = false;
                    auto [targetPt, bFront] = v;

                    // 横断歩道橋と重畳する道路の歩道中心線を収集
                    // 下段の処理で歩道中心線を更新する場合があるため、最新情報になるように都度収集する
                    SearchCenterLineRTree sclRtree; // 歩道中心線探索用RTree
                    for (const auto &group : map)
                    {
                        for (const auto &road : group.second)
                        {
                            if (road.nFunction == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH))
                            {
                                updateFootpathCenterLineRTree(sclRtree, road.tranDataPtr, dSamplingInterval);
                            }
                        }
                    }

                    if (sclRtree.size() > 0)
                    {
                        // 横断歩道橋の端点が歩道と重畳する場合
                        // 近傍歩道中心線の探索
                        std::vector<CenterLineTuple> nnResult;
                        sclRtree.query(bg::index::nearest(targetPt, 1), std::back_inserter(nnResult));

                        if (nnResult.size() > 0)
                        {
                            // lod3道路の場合は接続時に高さ確認をする?(誤接続対策)
                            auto [pt, pRoad, centerLineDataPtr, itPt, bFlag] = nnResult[0];
                            if (CEpsUtil::Less(pt.RoundDistance(targetPt), dDistTh))
                            {
                                bAddFlag = true;
                                if (bFront)
                                {
                                    // 横断歩道橋の始点側に接続点を追加
                                    centerLine.m_centerLine.centerLine.insert(centerLine.m_centerLine.centerLine.begin(), pt);
                                }
                                else
                                {
                                    // 横断歩道橋の終点側に接続点を追加
                                    centerLine.m_centerLine.centerLine.insert(centerLine.m_centerLine.centerLine.end(), pt);
                                }
                                if (bFlag)
                                {
                                    // サンプリング点の場合は近傍歩道中心線に点を追加する
                                    centerLineDataPtr->centerLine.insert(itPt + 1, pt);
                                }
                            }
                        }
                    }

                    if (!bAddFlag)
                    {
                        // 未接続横断歩道橋のため要確認メッセージを出力
                        CErrLogger::GetInstance()->WriteFootpathLog(
                            FootpathErrType::DISCONNECTED_PEDESTRIAN_BRIDGE, targetPt);
                    }
                }
            }
        }
    }
}

/*!
 * @brief 車道ネットワークの標高値設定
*/
void CNetworkCreator::SetRoadwayHeight()
{
    // 重畳道路探索用データの作成
    CSearchOverlapRoads sor;
    sor.SetData(m_tranRoadData, CInputSettingData::GetInstance()->lodType);

    std::vector<std::thread> threads;

    // 道路中心線
    for (auto tranRoadData : m_tranRoadData)
    {
        threads.emplace_back([tranRoadData, &sor, this]()
        {
            for (auto &centerLineDataPtr : tranRoadData->roadCenterLineList)
            {
                for (auto &pt : centerLineDataPtr->centerLine)
                {
                    CSearchOverlapRoads::ResultMap map = sor.Search(pt);
                    if (map.find(tranRoadData) != map.end())
                    {
                        std::set<double> height;
                        for (const auto &result : map[tranRoadData])
                        {
                            Boost3DPointHash crossPt;
                            if (CBoostGeoUtil::CalcVerticalCrossPt(*result.polygonPtr, pt, crossPt))
                            {
                                height.insert(crossPt.z());
                            }
                        }
                        if (height.size() > 0)
                            pt.z(*height.begin());  // TODO : 複数存在した場合はどうするか
                    }
                }
            }
        });
    }

    for (auto &th : threads)
        th.join();
}

/*!
 * @brief 歩道ネットワークの標高値設定
*/
void CNetworkCreator::SetFootpathHeight()
{
    // 重畳道路探索用データの作成
    CSearchOverlapRoads sor;
    sor.SetData(m_tranRoadData, CInputSettingData::GetInstance()->lodType);
    CSearchOverlapBridge sob;   // 橋梁用
    sob.SetData(m_bridgeData);

    std::vector<std::thread> threads;

    // 歩道中心線
    for (auto tranRoadData : m_tranRoadData)
    {
        threads.emplace_back([tranRoadData, &sor, this]()
        {
            for (auto &centerLineDataPtr : tranRoadData->m_footpath.centerLineList)
            {
                for (auto &pt : centerLineDataPtr->centerLine)
                {
                    CSearchOverlapRoads::ResultMap map = sor.Search(pt);
                    if (map.find(tranRoadData) != map.end())
                    {
                        std::set<double> height;
                        for (const auto &result : map[tranRoadData])
                        {
                            if (result.nFunction == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH))
                            {
                                // 対象を歩道ポリゴンに絞る
                                Boost3DPointHash crossPt;
                                if (CBoostGeoUtil::CalcVerticalCrossPt(*result.polygonPtr, pt, crossPt))
                                {
                                    height.insert(crossPt.z());
                                }
                            }
                        }
                        if (height.size() > 0)
                            pt.z(*height.begin());  // TODO : 複数存在した場合はどうするか
                    }
                }
            }
        });

    }

    // 横断歩道
    for (auto crossingPtr : m_pedestrianCrossingData)
    {
        threads.emplace_back([crossingPtr, &sor, this]()
        {
            if (crossingPtr->m_pedestrianCrossingData.m_bUse)
            {
                for (auto &pt : crossingPtr->m_pedestrianCrossingData.m_centerLineData.centerLine)
                {
                    CSearchOverlapRoads::ResultMap map = sor.Search(pt);
                    if (map.size() > 0)
                    {
                        std::set<double> height;
                        for (const auto &keyValue : map)
                        {
                            for (const auto &result : keyValue.second)
                            {
                                Boost3DPointHash crossPt;
                                if (CBoostGeoUtil::CalcVerticalCrossPt(*result.polygonPtr, pt, crossPt))
                                {
                                    height.insert(crossPt.z());
                                }
                            }
                        }
                        if (height.size() > 0)
                            pt.z(*height.begin());  // TODO : 複数存在した場合はどうするか
                    }
                }
            }
        });
    }

    // 横断歩道橋
    for (auto bridgePtr : m_bridgeData)
    {
        threads.emplace_back([bridgePtr, &sor, &sob, this]()
        {
            if (bridgePtr->m_pedestrianBridgeData.m_bUse)
            {
                for (auto &centerLine : bridgePtr->m_pedestrianBridgeData.m_centerLines)
                {
                    for (auto itPt = centerLine.m_centerLine.centerLine.begin();
                        itPt != centerLine.m_centerLine.centerLine.end(); itPt++)
                    {
                        // 歩道と接続する点は道路ポリゴンから、その他は橋梁ポリゴンから標高値を取得する

                        std::set<double> height;
                        if ((itPt == centerLine.m_centerLine.centerLine.begin() && !centerLine.m_bFrontLinkagePt)
                            || (itPt == centerLine.m_centerLine.centerLine.end() - 1 && !centerLine.m_bBackLinkagePt))
                        {
                            CSearchOverlapRoads::ResultMap map = sor.Search(*itPt);
                            if (map.size() > 0)
                            {
                                for (const auto &keyValue : map)
                                {
                                    for (const auto &result : keyValue.second)
                                    {
                                        Boost3DPointHash crossPt;
                                        if (CBoostGeoUtil::CalcVerticalCrossPt(*result.polygonPtr, *itPt, crossPt))
                                        {
                                            height.insert(crossPt.z());
                                        }
                                    }
                                }

                            }
                        }
                        else
                        {
                            CSearchOverlapBridge::ResultMap map = sob.Search(*itPt);
                            if (map.size() > 0)
                            {
                                for (const auto &keyValue : map)
                                {
                                    for (const auto &result : keyValue.second)
                                    {
                                        Boost3DPointHash crossPt;
                                        if (CBoostGeoUtil::CalcVerticalCrossPt(*result.polygonPtr, *itPt, crossPt))
                                        {
                                            height.insert(crossPt.z());
                                        }
                                    }
                                }

                            }
                        }
                        if (height.size() > 0)
                            itPt->z(*height.begin());  // TODO : 複数存在した場合はどうするか
                    }
                }
            }
        });
    }

    for (auto &th : threads)
        th.join();
}

/*!
 * @brief 歩道ネットワークデータの出力
 * @param[in] strShpOutputFolder        歩道SHP出力フォルダパス
 * @param[in] strGeoJsonOutputFolder    歩道GeoJSON出力フォルダパス
 * @param[in] nJPZone                   平面直角座標系の系番号
 * @param[in] bCreateSHP                SHP出力有無
 * @param[in] bCreateGeoJSON            GeoJSON出力有無
 * @param[in] dLod3Detail               LOD3の場合の詳細度
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
 */
bool CNetworkCreator::OutputFootpathNetwork(
    const std::string &strShpOutputFolder,
    const std::string &strGeoJsonOutputFolder,
    const int nJPZone,
    const bool bCreateSHP,
    const bool bCreateGeoJSON,
    const double dLod3Detail)
{
    if (!bCreateGeoJSON && !bCreateSHP)
        return false;

    CNetwork nw(CNetwork::NETWORK_DATA_TYPE::FOOTPATH, dLod3Detail);
    // 点字ブロックの情報を設定
    nw.SetBrailleTile(m_brailleBlocksData, 0.1, 1.0);
    // ネットワークデータの作成
    nw.Add(m_tranRoadData);
    nw.Add(m_pedestrianCrossingData);
    nw.Add(m_bridgeData);

    // 出力ファイルの作成
    CNetwork::OUTPUT_FILE_TYPE type;
    if (bCreateGeoJSON && bCreateSHP)
    {
        type = CNetwork::OUTPUT_FILE_TYPE::BOTH;
    }
    else if (bCreateGeoJSON)
    {
        type = CNetwork::OUTPUT_FILE_TYPE::GEOJSON;
    }
    else
    {
        type = CNetwork::OUTPUT_FILE_TYPE::SHP;
    }

    bool isUseZ = CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3 ? true : false;
    nw.OutputNetworkData(strShpOutputFolder, strGeoJsonOutputFolder, nJPZone, isUseZ, type);

    return true;
}

/*!
 * @brief ポリゴン外にポリラインがはみ出ているか確認する
 * @param[in] polyline  ポリライン
 * @param[in] polygon   マルチポリゴン
 * @param[in] dLengthTh 有効はみだし部分とみなすポリラインの長さしきい値
 * @return はみ出し部分のポリライン群(はみ出し部分がない場合は空マルチポリライン)
*/
Boost3DHashMultiLines CNetworkCreator::checkOutsideOfPolygon(
    const Boost3DHashPolyline &polyline,
    const Boost3DHashMultiPolygon &polygons,
    const double dLengthTh)
{
    Boost3DHashMultiLines debugLines;
    debugLines.push_back(polyline);

    Boost3DHashMultiLines outsideLines;
    BoostMultiPolygon srcPolys;
    for (const auto &poly : polygons)
        srcPolys.emplace_back(CBoostGeoUtil::Conv(poly));
    BoostPolyline srcLine = CBoostGeoUtil::Conv(polyline);
    BoostMultiLines dstLines;
    bg::difference(srcLine, srcPolys, dstLines);
    if (!bg::is_empty(dstLines))
    {
        for (const auto &line : dstLines)
        {
            if (CEpsUtil::GreaterEqual(bg::length(line), dLengthTh))
            {
                Boost3DHashPolyline tmpLine;
                for (const auto &pt : line)
                {
                    tmpLine.push_back(Boost3DPointHash(pt.x(), pt.y(), 0));
                }
                outsideLines.push_back(tmpLine);
            }
        }
    }
    return outsideLines;
}

/*!
 * @brief ポリゴン外にポリラインがはみ出ているか確認する(車道用)
 * @param[in] polyline  中心線
 * @param[in] roadPtr   道路情報ポインタ
 * @param[in] dLengthTh 有効はみだし部分とみなすポリラインの長さしきい値
 * @return はみ出し部分のポリライン群(はみ出し部分がない場合は空マルチポリライン)
*/
Boost3DHashMultiLines CNetworkCreator::checkOutsideOfPolygonForRoadway(
    const Boost3DHashPolyline &polyline,
    const std::shared_ptr<CTranRoadData> &roadPtr,
    const double dLengthTh)
{
    Boost3DHashMultiPolygon areaPolygons;
    if (m_iLod == 1)
    {
        areaPolygons.emplace_back(roadPtr->m_lod1.m_boostGeometry);
    }
    else if (m_iLod == 2)
    {
        Boost3DHashMultiPolygon tmp, island, footpath;
        for (const auto &lod2 : roadPtr->m_lod2List)
        {
            if (lod2.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY)
                || lod2.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION)
                || lod2.m_fuctionType == static_cast<std::underlying_type<AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE>::type>(AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND))
            {
                tmp.emplace_back(lod2.m_boostGeometry);
            }

            if (lod2.m_fuctionType == static_cast<std::underlying_type<AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE>::type>(AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND))
            {
                island.emplace_back(lod2.m_boostGeometry);
            }

            if (lod2.m_fuctionType == lod2.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH))
            {
                footpath.emplace_back(lod2.m_boostGeometry);
            }
        }

        // 島と隣接している歩道の探索
        for (const auto &footpathPoly : footpath)
        {
            for (const auto &islandPoly : island)
            {
                double dist = bg::distance(CBoostGeoUtil::Conv(footpathPoly), CBoostGeoUtil::Conv(islandPoly));
                if (CEpsUtil::Equal(dist, 0.0))
                {
                    tmp.emplace_back(footpathPoly);
                    break;
                }
            }
        }

        areaPolygons = CGDALUtil::Dissolve(tmp, false);
    }
    else if (m_iLod == 3)
    {
        Boost3DHashMultiPolygon tmp, island, footpath;
        for (const auto &lod3 : roadPtr->m_lod3List)
        {
            if (lod3.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY)
                || lod3.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::LANE)
                || lod3.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::ROADWAY_INTERSECTION)
                || lod3.m_fuctionType == static_cast<std::underlying_type<AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE>::type>(AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND))
            {
                tmp.emplace_back(lod3.m_boostGeometry);
            }

            if (lod3.m_fuctionType == static_cast<std::underlying_type<AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE>::type>(AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE::ISLAND))
            {
                island.emplace_back(lod3.m_boostGeometry);
            }

            if (lod3.m_fuctionType == lod3.m_fuctionType == static_cast<std::underlying_type<TRAFFIC_AREA_FUNCTION_TYPE>::type>(TRAFFIC_AREA_FUNCTION_TYPE::FOOTPATH))
            {
                footpath.emplace_back(lod3.m_boostGeometry);
            }
        }

        // 島と隣接している歩道の探索
        for (const auto &footpathPoly : footpath)
        {
            for (const auto &islandPoly : island)
            {
                double dist = bg::distance(CBoostGeoUtil::Conv(footpathPoly), CBoostGeoUtil::Conv(islandPoly));
                if (CEpsUtil::Equal(dist, 0.0))
                {
                    tmp.emplace_back(footpathPoly);
                    break;
                }
            }
        }

        areaPolygons = CGDALUtil::Dissolve(tmp, false);
    }

    Boost3DHashMultiLines outsideLines;
    if (!bg::is_empty(areaPolygons))
        outsideLines = checkOutsideOfPolygon(polyline, areaPolygons, dLengthTh);

    return outsideLines;
}

