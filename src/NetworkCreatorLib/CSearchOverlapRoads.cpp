#include "CSearchOverlapRoads.h"
#include "CBoostGeoUtil.h"

#pragma region 道路用

// コンストラクタ
CSearchOverlapRoads::CSearchOverlapRoads()
{
    m_bSetData = false;
}

// デストラクタ
CSearchOverlapRoads::~CSearchOverlapRoads()
{

}

// 探索用データセット一式のメモリ解放
void CSearchOverlapRoads::releaseData()
{
    if (!m_bSetData)
        return; // 未設定時は終了

    m_rtree.clear();
    m_bSetData = false;
}

// 道路情報の入力
void CSearchOverlapRoads::SetData(
    const std::vector<std::shared_ptr<CTranRoadData>> &tranData,
    const CInputSettingData::LODType &lodType)
{
    // 前回データの削除
    releaseData();

    // 指定されてLODによってRTreeに詰めるポリゴンを変更する
    for (const auto &tranPtr : tranData)
    {
        if (lodType == CInputSettingData::LODType::LOD1)
        {
            BoostPolygon poly = CBoostGeoUtil::Conv(tranPtr->m_lod1.m_boostGeometry);
            BoostBox box;
            bg::envelope(poly, box);
            CSearchOverlapRoadResult data;
            data.polygonPtr = std::make_shared<Boost3DHashPolygon>(tranPtr->m_lod1.m_boostGeometry);
            data.tranDataPtr = tranPtr;
            m_rtree.insert(std::pair(box, data));
        }
        else if (lodType == CInputSettingData::LODType::LOD2)
        {
            for (const auto &lod2 : tranPtr->m_lod2List)
            {
                BoostPolygon poly = CBoostGeoUtil::Conv(lod2.m_boostGeometry);
                BoostBox box;
                bg::envelope(poly, box);
                CSearchOverlapRoadResult data;
                data.polygonPtr = std::make_shared<Boost3DHashPolygon>(lod2.m_boostGeometry);
                data.tranDataPtr = tranPtr;
                data.nFunction = lod2.m_fuctionType;
                m_rtree.insert(std::pair(box, data));
            }
        }
        else if (lodType == CInputSettingData::LODType::LOD3)
        {
            // LOD3は三角メッシュを詰める
            for (const auto &lod3 : tranPtr->m_lod3TriangularMeshList)
            {
                BoostPolygon poly = CBoostGeoUtil::Conv(lod3.m_boostGeometry);
                BoostBox box;
                bg::envelope(poly, box);
                CSearchOverlapRoadResult data;
                data.polygonPtr = std::make_shared<Boost3DHashPolygon>(lod3.m_boostGeometry);
                data.tranDataPtr = tranPtr;
                data.nFunction = lod3.m_fuctionType;
                m_rtree.insert(std::pair(box, data));
            }
        }
    }

    m_bSetData = (m_rtree.size() > 0) ? true : false;
}

// 入力ポリゴンに2次元平面上で重畳するポリゴンの取得
CSearchOverlapRoads::ResultMap CSearchOverlapRoads::Search(const Boost3DHashPolygon &polygon)
{
    ResultMap result;

    BoostPolygon tmpPoly = CBoostGeoUtil::Conv(polygon);
    BoostBox box;
    bg::envelope(tmpPoly, box);
    std::vector<std::pair<BoostBox, CSearchOverlapRoadResult>> tmpResult;
    // bg::disjoint判定だとかなりシビアなので最近傍距離がしきい値以下は重畳と判断する
    m_rtree.query(
        bg::index::satisfies([tmpPoly](const std::pair<BoostBox, CSearchOverlapRoadResult> &v)
        { return CEpsUtil::LessEqual(bg::distance(tmpPoly, CBoostGeoUtil::Conv(*(v.second.polygonPtr))), 0.001); }),
        std::back_inserter(tmpResult));
    for (const auto &data : tmpResult)
    {
        if (result.find(data.second.tranDataPtr) == result.end())
        {
            // 新規登録
            std::vector<CSearchOverlapRoadResult> value;
            value.push_back(data.second);
            result.insert(std::make_pair(data.second.tranDataPtr, value));
        }
        else
        {
            // 更新
            result[data.second.tranDataPtr].push_back(data.second);
        }
    }

    return result;
}

// 入力頂点に2次元平面上で重畳するポリゴンの取得
CSearchOverlapRoads::ResultMap CSearchOverlapRoads::Search(const Boost3DPointHash &pt)
{
    ResultMap result;

    BoostPoint tmpPt = CBoostGeoUtil::Conv(pt);
    std::vector<std::pair<BoostBox, CSearchOverlapRoadResult>> tmpResult;
    // bg::disjoint判定だとかなりシビアなので最近傍距離がしきい値以下は重畳と判断する
    m_rtree.query(
        bg::index::satisfies([tmpPt](const std::pair<BoostBox, CSearchOverlapRoadResult> &v)
            { return CEpsUtil::LessEqual(bg::distance(tmpPt, CBoostGeoUtil::Conv(*(v.second.polygonPtr))), 0.001); }),
        std::back_inserter(tmpResult));
    for (const auto &data : tmpResult)
    {
        if (result.find(data.second.tranDataPtr) == result.end())
        {
            // 新規登録
            std::vector<CSearchOverlapRoadResult> value;
            value.push_back(data.second);
            result.insert(std::make_pair(data.second.tranDataPtr, value));
        }
        else
        {
            // 更新
            result[data.second.tranDataPtr].push_back(data.second);
        }
    }

    return result;
}
#pragma endregion 道路用

#pragma region 横断歩道橋用
// コンストラクタ
CSearchOverlapBridge::CSearchOverlapBridge()
{
    m_bSetData = false;
}

// デストラクタ
CSearchOverlapBridge::~CSearchOverlapBridge()
{

}

// 探索用データセット一式のメモリ解放
void CSearchOverlapBridge::releaseData()
{
    if (!m_bSetData)
        return; // 未設定時は終了

    m_rtree.clear();
    m_bSetData = false;
}

// 橋梁情報の入力
void CSearchOverlapBridge::SetData(
    const std::vector<std::shared_ptr<CBridgeData>> &bridData)
{
    // 前回データの削除
    releaseData();

    for (const auto &bridPtr : bridData)
    {
        for (const auto &lod2 : bridPtr->m_lod2TriangularMeshList)
        {
            BoostPolygon poly = CBoostGeoUtil::Conv(lod2.m_boostGeometry);
            BoostBox box;
            bg::envelope(poly, box);
            CSearchOverlapBridgeResult data;
            data.polygonPtr = std::make_shared<Boost3DHashPolygon>(lod2.m_boostGeometry);
            data.bridDataPtr = bridPtr;
            m_rtree.insert(std::pair(box, data));
        }
    }
    m_bSetData = (m_rtree.size() > 0) ? true : false;
}

// 入力頂点に2次元平面上で重畳するポリゴンの取得
CSearchOverlapBridge::ResultMap CSearchOverlapBridge::Search(const Boost3DPointHash &pt)
{
    ResultMap result;

    BoostPoint tmpPt = CBoostGeoUtil::Conv(pt);
    std::vector<std::pair<BoostBox, CSearchOverlapBridgeResult>> tmpResult;
    // bg::disjoint判定だとかなりシビアなので最近傍距離がしきい値以下は重畳と判断する
    m_rtree.query(
        bg::index::satisfies([tmpPt](const std::pair<BoostBox, CSearchOverlapBridgeResult> &v)
            { return CEpsUtil::LessEqual(bg::distance(tmpPt, CBoostGeoUtil::Conv(*(v.second.polygonPtr))), 0.001); }),
        std::back_inserter(tmpResult));
    for (const auto &data : tmpResult)
    {
        if (result.find(data.second.bridDataPtr) == result.end())
        {
            // 新規登録
            std::vector<CSearchOverlapBridgeResult> value;
            value.push_back(data.second);
            result.insert(std::make_pair(data.second.bridDataPtr, value));
        }
        else
        {
            // 更新
            result[data.second.bridDataPtr].push_back(data.second);
        }
    }

    return result;
}
#pragma endregion 横断歩道橋用
