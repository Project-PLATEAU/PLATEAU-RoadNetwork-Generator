#include "CSearchNeighbor.h"
#include "CGeoUtil.h"
#include "CGDALUtil.h"

// コンストラクタ
CSearchNeighbor::CSearchNeighbor()
{
    m_bSetData = false;
    m_iLod = 1;
    m_dLodType = 3.0;
}

// コンストラクタ
CSearchNeighbor::CSearchNeighbor(int lod, double lodType)
{
    m_bSetData = false;
    m_iLod = lod;
    m_dLodType = lodType;
}

// デストラクタ
CSearchNeighbor::~CSearchNeighbor()
{
    // メモリ開放
    releaseData();
}

// 探索用データセット一式のメモリ解放
void CSearchNeighbor::releaseData()
{
    if (!m_bSetData)
        return; // 未設定時は終了

    m_neighborMap.clear();
    m_hashMap.clear();
    m_segmentMap.clear();
    m_bSetData = false;
}

// ハッシュマップの更新
void CSearchNeighbor::updateMap(
    const Boost3DHashRing &ring,
    const std::shared_ptr<CTranRoadData> &roadPtr,
    Boost3DPointHashMap &map)
{
    if (ring.size() < 4)
        return;

    for (size_t i = 0; i < ring.size() - 1; i++)
    {
        // 注目頂点と前後点を結ぶ辺のなす角
        CVector2D prevPt(ring[ring.size() - 2].x(), ring[ring.size() - 2].y());    // 終点は始点と同一点のため
        if (i > 0)
            prevPt = CVector2D(ring[i - 1].x(), ring[i - 1].y());
        CVector2D nextPt(ring[i + 1].x(), ring[i + 1].y());
        CVector2D currentPt(ring[i].x(), ring[i].y());
        CVector2D vec1 = prevPt - currentPt;
        CVector2D vec2 = nextPt - currentPt;
        double dAngle = CGeoUtil::Angle(vec1, vec2);

        Boost3DPointHash pt = ring[i];
        Boost3DPointHashMap::iterator it = map.find(pt);
        if (it == map.end())
        {
            // 新規追加
            Boost3DPointHashMapValue val;
            val.insert(Boost3DPointHashMapValue::value_type(roadPtr, dAngle));
            map.insert(Boost3DPointHashMap::value_type(pt, val));
        }
        else
        {
            // 更新
            Boost3DPointHashMapValue::iterator itValue = it->second.find(roadPtr);
            if (itValue == it->second.end())
                it->second.insert(Boost3DPointHashMapValue::value_type(roadPtr, dAngle));
        }
    }
}

// 近傍道路マップの更新
void CSearchNeighbor::updateNearlyRoadMap(
    const Boost3DHashRing &ring,
    const std::shared_ptr<CTranRoadData> &roadPtr,
    const Boost3DPointHashMap &hashMap,
    std::unordered_map<std::shared_ptr<CTranRoadData>,
        std::set<std::shared_ptr<CTranRoadData>>> &map)
{
    for (const auto &pt : ring)
    {
        auto it = hashMap.find(pt);
        if (it != hashMap.end())
        {
            auto itRoad = map.find(roadPtr);
            if (itRoad == map.end())
            {
                // 新規追加
                std::set<std::shared_ptr<CTranRoadData>> val;
                for (const auto &otherRoadPtr : it->second)
                {
                    if (otherRoadPtr.first != roadPtr)
                        val.insert(otherRoadPtr.first);
                }
                map.insert(std::pair<std::shared_ptr<CTranRoadData>,
                    std::set<std::shared_ptr<CTranRoadData>>>(roadPtr, val));
            }
            else
            {
                // 更新
                for (const auto &otherRoadPtr : it->second)
                {
                    if (otherRoadPtr.first != roadPtr)
                        itRoad->second.insert(otherRoadPtr.first);
                }
            }
        }
    }
}

// 隣接道路マップの更新
void CSearchNeighbor::updateNeighborRoadMap(
    const std::shared_ptr<CTranRoadData> &targetPtr,
    const std::shared_ptr<CTranRoadData> &otherPtr,
    const Boost3DPointHashMap &hashMap,
    const SegmentMap &segmentMap,
    std::unordered_map<std::shared_ptr<CTranRoadData>,
    std::set<std::shared_ptr<CTranRoadData>>> &map)
{
    const double dLengthTh = 150.0;     // 延長方向のセグメントとみなす長さしきい値
    const double dDiffLengthTh = 0.1;   // 重畳セグメントとみなすセグメント長さ差分のしきい値
    const double dRoadWidthTh = 1.0;    // 進入進出口とみなすセグメント長さのしきい値
    if (map.find(targetPtr) != map.end())
    {
        if (map[targetPtr].find(otherPtr) != map[targetPtr].end())
            return; // 更新済みデータのためスキップ
    }

    if (bg::covered_by(targetPtr->m_lod1.m_boostGeometry, otherPtr->m_lod1.m_boostGeometry)
        || bg::covered_by(otherPtr->m_lod1.m_boostGeometry, targetPtr->m_lod1.m_boostGeometry))
    {
        // 内包している場合はスキップ
        return;
    }

    const auto itTargetSegment = segmentMap.find(targetPtr);
    const auto itOtherSegment = segmentMap.find(otherPtr);
    bool bNeighbor = false;
    if (itTargetSegment != segmentMap.end() && itOtherSegment != segmentMap.end())
    {
        for (const auto &polyline : itTargetSegment->second)
        {
            if (CEpsUtil::GreaterEqual(bg::length(polyline), dLengthTh))
                continue;   // 長いセグメントは延長方向のエッジとみなす

            // 共有セグメントの探索
            for (const auto &otherPolyline : itOtherSegment->second)
            {
                if (CEpsUtil::GreaterEqual(bg::length(otherPolyline), dLengthTh))
                    continue;   // 長いセグメントは延長方向のエッジとみなす

                Boost3DHashPolyline::const_iterator itStart, itEnd, itUnknown;
                double dShortPathLength = 0;
                if (CEpsUtil::Less(bg::length(polyline), bg::length(otherPolyline)))
                {
                    // 注目道路のセグメントが短い場合
                    dShortPathLength = bg::length(polyline);
                    itUnknown = otherPolyline.cend();
                    itStart = itUnknown;
                    itEnd = itUnknown;
                    for (Boost3DHashPolyline::const_iterator itPt = otherPolyline.cbegin();
                        itPt != otherPolyline.cend(); itPt++)
                    {
                        if (polyline.front().IsRoundEqual(*itPt))
                            itStart = itPt;

                        if (polyline.back().IsRoundEqual(*itPt))
                            itEnd = itPt;
                    }
                }
                else
                {
                    // 注目道路のセグメントが長い場合
                    dShortPathLength = bg::length(otherPolyline);
                    itUnknown = polyline.cend();
                    itStart = itUnknown;
                    itEnd = itUnknown;
                    for (Boost3DHashPolyline::const_iterator itPt = polyline.cbegin();
                        itPt != polyline.cend(); itPt++)
                    {
                        if (otherPolyline.front().IsRoundEqual(*itPt))
                            itStart = itPt;

                        if (otherPolyline.back().IsRoundEqual(*itPt))
                            itEnd = itPt;
                    }
                }

                if (itStart != itUnknown && itEnd != itUnknown)
                {
                    Boost3DHashPolyline path;
                    if (itStart < itEnd)
                    {
                        for (Boost3DHashPolyline::const_iterator itPt = itStart; itPt <= itEnd; itPt++)
                            path.push_back(*itPt);
                    }
                    else
                    {
                        for (Boost3DHashPolyline::const_iterator itPt = itEnd; itPt <= itStart; itPt++)
                            path.push_back(*itPt);
                    }
                    double dLongPathLength = bg::length(path);
                    double dDiff = abs(dLongPathLength - dShortPathLength);
                    if (CEpsUtil::GreaterEqual(dShortPathLength, dRoadWidthTh)
                        && CEpsUtil::Less(dDiff, dDiffLengthTh))
                    {
                        // 進入進出口の発見
                        targetPtr->m_nInOut += 1;
                        otherPtr->m_nInOut += 1;
                        bNeighbor = true;
                        break;
                    }

                }
            }
        }
    }

    if (bNeighbor)
    {
        // 共有する辺が存在する場合は隣接道路と判断
        auto itTarget = map.find(targetPtr);
        if (itTarget != map.end())
        {
            itTarget->second.insert(otherPtr);
        }
        else
        {
            std::set<std::shared_ptr<CTranRoadData>> val;
            val.insert(otherPtr);
            map.insert(std::pair<std::shared_ptr<CTranRoadData>,
                std::set<std::shared_ptr<CTranRoadData>>>(targetPtr, val));
        }

        auto itOther = map.find(otherPtr);
        if (itOther != map.end())
        {
            itOther->second.insert(targetPtr);
        }
        else
        {
            std::set<std::shared_ptr<CTranRoadData>> val;
            val.insert(targetPtr);
            map.insert(std::pair<std::shared_ptr<CTranRoadData>,
                std::set<std::shared_ptr<CTranRoadData>>>(otherPtr, val));
        }
    }
}

// 道路外周線を特徴点で分割してセグメントを作成しセグメントマップを更新する
void CSearchNeighbor::updateSegmentMap(
    const Boost3DHashRing &ring,
    const std::shared_ptr<CTranRoadData> &targetPtr,
    const Boost3DPointHashMap &hashMap,
    const std::unordered_map<std::shared_ptr<CTranRoadData>,
        std::set<std::shared_ptr<CTranRoadData>>> &map,
    SegmentMap &segmentMap)
{
    if (ring.size() < 1)
        return;

    const double dHorizonAngle = 170.0; // 水平線の角度しきい値

    // セグメント開始点の探索
    size_t normalStartIdx = 0;
    size_t nearlyStartIdx = 0;
    bool bNormalStart = false;  // 通常開始点発見フラグ
    bool bNearlyStart = false;  // 近傍道路有り時の開始点発見フラグ
    for (size_t idx = 0; idx < ring.size() - 1; idx++)
    {
        double dAngle;
        if (getPtAngle(ring[idx], targetPtr, hashMap, dAngle))
        {
            if (CEpsUtil::Less(dAngle, dHorizonAngle))
            {
                if (!bNormalStart)
                {
                    normalStartIdx = idx;
                    bNormalStart = true;
                }

                if (!bNearlyStart)
                {
                    // 近傍道路が存在する場合は共有点を開始点とする
                    const auto &it = hashMap.find(ring[idx]);
                    if (it != hashMap.end() && it->second.size() > 1)
                    {
                        nearlyStartIdx = idx;
                        bNearlyStart = true;
                    }
                }
            }
        }

        if (bNormalStart && bNearlyStart)
            break;
    }

    size_t startIdx = (bNearlyStart) ? nearlyStartIdx : normalStartIdx;
    size_t currentIdx = startIdx;
    bool bStart = true;
    do
    {
        Boost3DHashPolyline polyline;
        do
        {
            polyline.push_back(ring[currentIdx]);
            if (bStart)
            {
                bStart = false;
            }
            else
            {
                // 終了判定
                const auto &it = hashMap.find(ring[currentIdx]);
                if (it != hashMap.end() && it->second.size() > 1)
                {
                    double dAngle;
                    if (getPtAngle(ring[currentIdx], targetPtr, hashMap, dAngle))
                    {
                        if (CEpsUtil::Less(dAngle, dHorizonAngle))
                        {
                            // 近傍ポリゴンとの共有点かつ前後点を結ぶ辺のなす角が水平ではない場合
                            break;
                        }
                    }
                }
            }

            currentIdx++;
            if (currentIdx >= ring.size() - 1)
                currentIdx = 0; // 終点は始点と同一のためskip

        } while (currentIdx != startIdx);

        if (currentIdx == startIdx)
            polyline.push_back(ring[currentIdx]);   // 終点追加

        const auto &it = segmentMap.find(targetPtr);
        if (it == segmentMap.end())
        {
            // 新規登録
            Boost3DHashMultiLines lines;
            lines.push_back(polyline);
            segmentMap.insert(SegmentMap::value_type(targetPtr, lines));

        }
        else
        {
            // 更新
            it->second.push_back(polyline);
        }
        bStart = true;
    } while (currentIdx != startIdx);
}

 // ハッシュマップから指定道路の頂点角度を取得する
bool CSearchNeighbor::getPtAngle(
    const Boost3DPointHash &pt,
    const std::shared_ptr<CTranRoadData> &targetPtr,
    const Boost3DPointHashMap &hashMap,
    double &dAngle)
{
    bool bRet = false;
    const auto &it = hashMap.find(pt);
    if (it != hashMap.end())
    {
        const auto &itVal = it->second.find(targetPtr);
        if (itVal != it->second.end())
        {
            bRet = true;
            dAngle = itVal->second;
        }
    }
    return bRet;
}

// 道路情報の入力
void CSearchNeighbor::SetData(std::vector<std::shared_ptr<CTranRoadData>> &tranData)
{
    // LOD3の対応
    // 現状LOD1で判定しているので、立体交差部分の隣接判定が怪しい
    std::vector<std::pair<std::shared_ptr<CTranRoadData>, Boost3DHashPolygon>> tranRoadDataPolygonPairList;
    for (const auto& ptr : tranData)
    {
        bool isExistPolygon = false;
        switch (m_iLod)
        {
        case 2:
            if (ptr->m_lod2List.size() > 0)
            {
                isExistPolygon = true;
            }
            break;
        case 3:
            if (ptr->m_lod3List.size() > 0)
            {
                isExistPolygon = true;
            }
            break;
        case 1:
        default:
            isExistPolygon = true;
            break;
        }

        if (isExistPolygon)
        {
            tranRoadDataPolygonPairList.emplace_back(std::make_pair(ptr, ptr->m_lod1.m_boostGeometry));
        }
    }

    // 前回データを削除
    releaseData();

    // ハッシュマップによる頂点座標に紐づく道路ポインタの整理
    for (const auto& pair : tranRoadDataPolygonPairList)
    {
        // 外輪郭の頂点によるマップ更新
        updateMap(pair.second.outer(), pair.first, m_hashMap);

        for (const auto& inner : pair.second.inners())
        {
            // 内輪郭の頂点によるマップ更新
            updateMap(inner, pair.first, m_hashMap);
        }
    }

    // 近傍ポリゴン判定
    std::unordered_map<std::shared_ptr<CTranRoadData>,
        std::set<std::shared_ptr<CTranRoadData>>> nearlyRoadMap;   // 近傍ポリゴン関係マップ
    for (const auto& pair : tranRoadDataPolygonPairList)
    {
        updateNearlyRoadMap(pair.second.outer(), pair.first, m_hashMap, nearlyRoadMap);
        for (const auto& inner : pair.second.inners())
        {
            updateNearlyRoadMap(inner, pair.first, m_hashMap, nearlyRoadMap);
        }
    }

    // 道路外周線のセグメント分割
    for (const auto& pair : tranRoadDataPolygonPairList)
    {
        // 外輪郭の頂点によるマップ更新
        updateSegmentMap(
            pair.second.outer(), pair.first, m_hashMap, nearlyRoadMap, m_segmentMap);
    }

    // 近隣ポリゴン関係マップを基に隣接ポリゴン判定
    for (const auto& pair : tranRoadDataPolygonPairList)
    {
        if (nearlyRoadMap.find(pair.first) != nearlyRoadMap.end())
        {
            for (const auto& otherPtr : nearlyRoadMap[pair.first])
            {
                updateNeighborRoadMap(pair.first, otherPtr, m_hashMap, m_segmentMap, m_neighborMap);
            }
        }
    }

    // 道路情報に隣接道路情報をセット
    for (const auto& pair : tranRoadDataPolygonPairList)
    {
        if (m_neighborMap.find(pair.first) != m_neighborMap.end())
        {
            pair.first->m_neighborRoadPtr = m_neighborMap[pair.first];
        }
    }
    m_bSetData = true;
}
