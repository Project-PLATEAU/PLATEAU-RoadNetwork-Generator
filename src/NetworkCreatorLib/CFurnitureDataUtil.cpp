#include "CFurnitureDataUtil.h"
#include "CConvertGeometryUtil.h"
#include "CGDALUtil.h"
#include "CBoostGeoUtil.h"
#include "boost/foreach.hpp"
#include <algorithm>

// 都市設備品質情報のlodType(LOD3の場合の詳細度)を取得する
bool CFurnitureDataUtil::GetFurnitureDataQualityAttributeLodType(
    const citygml::CityObject *pFurniture, double &dLodType)
{
    bool bRet = false;

    if (pFurniture != nullptr)
    {
        auto it = pFurniture->getAttributes().find(KEY_FRN_DATA_QUALITY_ATTR);
        if (it != pFurniture->getAttributes().end())
        {
            for (auto val : it->second.asAttributeSet())
            {
                if (val.first == KEY_FRN_DATA_QUALITY_ATTR_LOD_TYPE)
                {
                    bRet = true;
                    dLodType = val.second.asDouble();
                    break;
                }
            }
        }
    }

    return bRet;
}

// CityObjectからfrn:functionを取得
bool CFurnitureDataUtil::GetFurnitureFunction(
    const citygml::CityObject *pFurniture,
    CFurnitureData &furnitureData)
{
    bool bRet = false;
    furnitureData.m_functionType = FURNITURE_FUNCTION_TYPE::UNKNOWN;

    if (pFurniture != nullptr)
    {
        auto it = pFurniture->getAttributes().find(KEY_FRN_FUNCTION);
        if (it != pFurniture->getAttributes().end())
        {
            std::string strFunction = it->second.asString();
            int nFunction = atoi(strFunction.c_str());

            for (FURNITURE_FUNCTION_TYPE type : FURNITURE_FUNCTION_TYPE())
            {
                if (static_cast<std::underlying_type<FURNITURE_FUNCTION_TYPE>::type>(type) == nFunction)
                {
                    furnitureData.m_functionType = type;
                    bRet = true;
                    break;
                }
            }
        }
    }
    return bRet;
}

// LOD1,2,3幾何情報の取得
bool CFurnitureDataUtil::GetGeometry(
    const citygml::CityObject *pFurniture,
    CFurnitureData &furnitureData,
    const int nJPZone)
{
    const int NUM = 3;
    bool bRet = false;
    if (pFurniture != nullptr)
    {
        Boost3DHashMultiPolygon polys[NUM];
        for (uint32_t geoIdx = 0; geoIdx < pFurniture->getGeometriesCount(); geoIdx++)
        {
            auto geo = pFurniture->getGeometry(geoIdx);

            if (geo.getPolygonsCount() > 0)
            {
                for (uint32_t polyIdx = 0; polyIdx < geo.getPolygonsCount(); polyIdx++)
                {
                    std::shared_ptr<const citygml::Polygon> pPoly = geo.getPolygon(polyIdx);
                    auto polygon = CConvertGeometryUtil::ConvBoostPolygon(pPoly, nJPZone);

                    if (!bg::is_empty(polygon))
                    {
                        if (geo.getLOD() == static_cast<uint32_t>(LOD_TYPE::LOD1))
                        {
                            polys[0].push_back(polygon);
                        }
                        else if (geo.getLOD() == static_cast<uint32_t>(LOD_TYPE::LOD2))
                        {
                            polys[1].push_back(polygon);
                        }
                        else if (geo.getLOD() == static_cast<uint32_t>(LOD_TYPE::LOD3))
                        {
                            polys[2].push_back(polygon);
                        }
                    }
                }
            }
        }

        for (size_t i = 0; i < NUM; i++)
        {
            Boost3DHashMultiPolygon dissolvePolygons;
            if (polys[i].size() > 1)
            {
                // 外形形状を取得したいため融解する
                dissolvePolygons = CGDALUtil::GetInstance()->Dissolve(polys[i], true, false);
            }
            else
            {
                dissolvePolygons = polys[i];
            }

            if (!bg::is_empty(dissolvePolygons))
            {
                int nLod = i + 1;
                for (const auto &poly : dissolvePolygons)
                {
                    if (nLod == static_cast<std::underlying_type<FURNITURE_FUNCTION_TYPE>::type>(LOD_TYPE::LOD1))
                    {
                        CFurnitureDataLod1 lod1Data;
                        lod1Data.m_boostGeometry = poly;
                        furnitureData.m_lod1List.emplace_back(lod1Data);
                    }
                    else if (nLod == static_cast<std::underlying_type<FURNITURE_FUNCTION_TYPE>::type>(LOD_TYPE::LOD2))
                    {
                        CFurnitureDataLod2 lod2Data;
                        lod2Data.m_boostGeometry = poly;
                        furnitureData.m_lod2List.emplace_back(lod2Data);
                    }
                    else if (nLod == static_cast<std::underlying_type<FURNITURE_FUNCTION_TYPE>::type>(LOD_TYPE::LOD3))
                    {
                        CFurnitureDataLod3 lod3Data;
                        lod3Data.m_boostGeometry = poly;
                        furnitureData.m_lod3List.emplace_back(lod3Data);
                    }
                }
            }
        }

        bRet = (furnitureData.m_lod1List.size() > 0
            || furnitureData.m_lod2List.size() > 0
            || furnitureData.m_lod3List.size() > 0);
    }
    return bRet;
}


// 横断歩道と点字ブロックの場合CityObjectから都市設備データを取得する
bool CFurnitureDataUtil::GetPedestrianCrossingOrBrailleBlocks(
    const citygml::CityObject *pFurniture,
    CFurnitureData &furnitureData,
    const int nJPZone)
{
    bool bRet = false;
    furnitureData.m_strId = pFurniture->getId();
    if (GetFurnitureFunction(pFurniture, furnitureData))
    {
        if (furnitureData.m_functionType == FURNITURE_FUNCTION_TYPE::PEDESTRIAN_CROSSING
            || furnitureData.m_functionType == FURNITURE_FUNCTION_TYPE::BRAILLE_BLOCKS)
        {
            // 横断歩道 or 点字ブロック
            bRet = GetGeometry(pFurniture, furnitureData, nJPZone);
        }
    }

    return bRet;
}

// 横断歩道の中心線の作成
void CFurnitureDataUtil::CreateCenterLineOfPedestrianCrossing(
    std::shared_ptr<CFurnitureData> &pFurniture,
    const double dParallelEpsilon)
{
    // 横断歩道の縞形状確認
    const double dLengthDiffTh = 0.1;       // 長さ差分の誤差
    const double dMargin = 0.1;             // マージン距離
    const double dAngleStvTh = 10.0;        // 縞タイプ判定用の角度標準偏差しきい値
    Boost3DHashMultiPolygon srcPolygons;    // 入力ポリゴン群
    BoostMultiPolygon simplePolygons;       // 入力ポリゴンの簡略化ポリゴン群
    BoostMultiPolygon validPolygons;        // 横断歩道の方向算出で使用するポリゴン群
    Boost3DHashMultiPolygon mbrs;           // 横断歩道の方向算出で使用するポリゴンのMBR
    CRotateAngleVecDataManager rotMngForCheckType;  // 横断歩道の種別判定用
    CRotateAngleVecDataManager rotMngForStripe;     // 縞タイプの回転方向決定用

    // 使用するポリゴンの選択(LOD3優先)
    if (pFurniture->m_lod3List.size() > 0)
    {
        for (const auto &data : pFurniture->m_lod3List)
            srcPolygons.push_back(data.m_boostGeometry);
    }
    else if (pFurniture->m_lod2List.size() > 0)
    {
        for (const auto &data : pFurniture->m_lod2List)
            srcPolygons.push_back(data.m_boostGeometry);
    }

    // 前処理
    preprocessForPedestrianCrossing(
        srcPolygons, simplePolygons, validPolygons, mbrs,
        rotMngForCheckType, rotMngForStripe);


    // 横断歩道の各ジオメトリの最長辺の内、最長/最短辺の長さを取得する
    // 縞と自転車横断歩道帯が混在していると長さのばらつきが大きく、スクランブル交差点だとばらつきが小さい
    double dLong = 0;
    double dShort = DBL_MAX;
    for (const auto &d : rotMngForCheckType.data)
    {
        if (d.nCount > 1)
        {
            if (CEpsUtil::Less(d.dLength, dShort))
                dShort = d.dLength;
            if (CEpsUtil::Greater(d.dLength, dLong))
                dLong = d.dLength;
        }
    }

    // 長辺方向のばらつき具合を確認(縞と自転車横断歩道帯が混在していると長辺方向がばらつく)
    std::vector<double> degrees;
    for (const auto &d : rotMngForCheckType.data)
    {
        for (int t = 0; t < d.nCount; t++)
            degrees.push_back(d.dAngle);
    }
    double dAve, dVar, dStd;
    if (calcParam(degrees, dAve, dVar, dStd))
    {
        pFurniture->m_pedestrianCrossingData.m_dAve = dAve;
        pFurniture->m_pedestrianCrossingData.m_dVar = dVar;
        pFurniture->m_pedestrianCrossingData.m_dStd = dStd;
    }

    // 最長の長辺方向の取得(縦線タイプの場合の横断歩道の進行方向)
    auto itLongVec = rotMngForCheckType.data.begin();
    for (auto it = rotMngForCheckType.data.begin(); it < rotMngForCheckType.data.end(); it++)
    {
        if (CEpsUtil::Less(itLongVec->dLength, it->dLength))
            itLongVec = it;
    }
    // 縦線タイプか確認する
    // 縦線タイプの場合、前段の最長の長辺方向を基準に横断歩道のMBRを取得した際に、
    // 長辺付近に横断歩道のポリゴン(自転車マークは除く)が集約されるはず
    BoostMultiPolygon rects;
    int nCoverdPolyCount = 0;
    bool bRects[2] = { false, false };
    if (makeRects(rects, simplePolygons, itLongVec->vec, itLongVec->pos))
    {
        for (auto &rect : rects)
            rect = CBoostGeoUtil::Buffering(rect, dMargin); // マージン追加

        for (const auto &poly : validPolygons)
        {
            if (bg::covered_by(poly, rects[0]))
            {
                nCoverdPolyCount++;
                bRects[0] = true;
            }
            else if (bg::covered_by(poly, rects[1]))
            {
                nCoverdPolyCount++;
                bRects[1] = true;
            }
        }
    }
    if (nCoverdPolyCount == static_cast<int>(validPolygons.size())
        && bRects[0] && bRects[1])
    {
        pFurniture->m_pedestrianCrossingData.m_bStripes = false; // 縦線タイプ
    }
    else
    {
        if (CEpsUtil::LessEqual(pFurniture->m_pedestrianCrossingData.m_dStd, dAngleStvTh))
        {
            // 進行方向が1方向の縞タイプ
            pFurniture->m_pedestrianCrossingData.m_bStripes = true;  // 縞タイプ
        }
        else
        {
            // 縞と縦線タイプが混在 or スクランブル交差点(複数方向の縞タイプが混在)
            double dDiff = abs(dLong - dShort);
            if (CEpsUtil::LessEqual(dDiff, dLengthDiffTh))
            {
                // スクランブル交差点
                pFurniture->m_pedestrianCrossingData.m_bStripes = true;  // 縞タイプ
            }
            else
            {
                // 縞と縦線タイプが混在
                pFurniture->m_pedestrianCrossingData.m_bStripes = false; // 縦線タイプ
            }
        }
    }

    if (!pFurniture->m_pedestrianCrossingData.m_bStripes)
    {
        // 横断歩道の進行方向は最長の長辺方向とする
        pFurniture->m_pedestrianCrossingData.m_directionOfMovement = itLongVec->vec;
        pFurniture->m_pedestrianCrossingData.m_rotateCenter = itLongVec->pos;
    }
    else
    {
        if (rotMngForStripe.data.size() > 1)
        {
            // 横断歩道の縞々形状の長辺と短辺が抽出されるはずのため
            // 頻出数上位2種類のデータの内、辺の長さが短い方を採用する
            // 想定外の向きが取得される場合は頻出数が少ないはず
            size_t t = rotMngForStripe.data.size() - 1;
            pFurniture->m_pedestrianCrossingData.m_directionOfMovement = (rotMngForStripe.data[t].dLength < rotMngForStripe.data[t - 1].dLength) ? rotMngForStripe.data[t].vec : rotMngForStripe.data[t - 1].vec;
            pFurniture->m_pedestrianCrossingData.m_rotateCenter = (rotMngForStripe.data[t].dLength < rotMngForStripe.data[t - 1].dLength) ? rotMngForStripe.data[t].pos : rotMngForStripe.data[t - 1].pos;
        }
        else if (rotMngForStripe.data.size() != 0)
        {
            pFurniture->m_pedestrianCrossingData.m_directionOfMovement = rotMngForStripe.data.back().vec;
            pFurniture->m_pedestrianCrossingData.m_rotateCenter = rotMngForStripe.data.back().pos;
        }
    }

    // 中心線作成
    if (simplePolygons.size() > 0)
    {
        double dShort, dLong;
        Boost3DHashPolygon mbr;
        if (CBoostGeoUtil::MBR(simplePolygons, pFurniture->m_pedestrianCrossingData.m_directionOfMovement,
                pFurniture->m_pedestrianCrossingData.m_rotateCenter, mbr, dShort, dLong))
        {
            Boost3DHashPolyline centerline;
            double dWidth = dLong;  // 幅員の初期値はMBRの長辺とする(道路幅の方が長くて進行方向が短い場合を考慮)
            for (size_t t = 0; t < mbr.outer().size() - 1; t++)
            {
                CVector2D pt1(mbr.outer().at(t).x(), mbr.outer().at(t).y());
                CVector2D pt2(mbr.outer().at(t + 1).x(), mbr.outer().at(t + 1).y());
                CVector2D tmpVec = pt2 - pt1;
                double dAngle1 = CGeoUtil::Angle(pFurniture->m_pedestrianCrossingData.m_directionOfMovement, tmpVec);
                double dAngle2 = CGeoUtil::Angle(-1 * pFurniture->m_pedestrianCrossingData.m_directionOfMovement, tmpVec);

                // 進行方向に並行であるか確認する
                bool bParallel = !(CEpsUtil::Greater(dAngle1, dParallelEpsilon)
                    && CEpsUtil::Greater(dAngle2, dParallelEpsilon));

                if (!bParallel)
                {
                    // 進行方向に非平行な場合
                    CVector2D tmpPt = 0.5 * tmpVec + pt1;
                    centerline.push_back(Boost3DPointHash(tmpPt.x, tmpPt.y, 0));

                    // 進行方向と非並行なMBRの辺の長さを横断歩道の幅員とする
                    if (CEpsUtil::Less(tmpVec.Length(), dWidth))
                    {
                        dWidth = tmpVec.Length();
                    }
                }
            }
            if (!bg::is_empty(centerline))
            {
                pFurniture->m_pedestrianCrossingData.m_centerLineData.centerLine = centerline;
                pFurniture->m_pedestrianCrossingData.m_mbr = mbr;
                pFurniture->m_pedestrianCrossingData.m_centerLineData.dMinWidth = dWidth;
                Boost3DPointHash centerPt;
                bg::centroid(mbr, centerPt);
                pFurniture->m_pedestrianCrossingData.m_centerLineData.minWidthPos = centerPt;
            }
        }
    }

    //// debug
    //pFurniture->m_pedestrianCrossingData.m_src = srcPolygons;
    //pFurniture->m_pedestrianCrossingData.m_simple = simplePolygons;
    //pFurniture->m_pedestrianCrossingData.m_mbrs = mbrs;
    //pFurniture->m_pedestrianCrossingData.m_rects = rects;

}

// 横断歩道の重複確認
// 歩行者横断歩道と自転車横断歩道が隣接している場合は自転車横断歩道を使用しない設定に変更する
void CFurnitureDataUtil::DuplicateCheck(
    std::vector<std::shared_ptr<CFurnitureData>> &furnitures,
    const double dDistTh)
{
    typedef bg::index::rtree<
        std::pair<Boost3DPointHash, std::shared_ptr<CFurnitureData>>, bg::index::quadratic<16>> SearchNNCrossingRTree;
    SearchNNCrossingRTree snncRtree;
    for (auto &crossing : furnitures)
    {
        // 横断歩道の中心線の中点をRTreeに登録(近傍探索用)
        Boost3DPointHash centerPt = CBoostGeoUtil::GetCenterPt(
            crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.front(),
            crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.back());
        snncRtree.insert(std::make_pair(centerPt, crossing));
    }
    for (auto &crossing : furnitures)
    {
        if (crossing->m_pedestrianCrossingData.m_bUse)
        {
            // 使用予定の中心線の場合
            Boost3DPointHash centerPt = CBoostGeoUtil::GetCenterPt(
                crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.front(),
                crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.back());

            // 近傍探索
            std::vector<std::pair<Boost3DPointHash, std::shared_ptr<CFurnitureData>>> nnResult;
            snncRtree.query(bg::index::nearest(centerPt, 2), std::back_inserter(nnResult));
            CVector2D vec2d(
                crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.back().x()
                    - crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.front().x(),
                crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.back().y()
                    - crossing->m_pedestrianCrossingData.m_centerLineData.centerLine.front().y());
            for (auto &neighbor : nnResult)
            {
                if (neighbor.second->m_strId != crossing->m_strId
                    && neighbor.second->m_pedestrianCrossingData.m_bStripes != crossing->m_pedestrianCrossingData.m_bStripes)
                {
                    // 注目横断歩道以外の横断歩道かつ横断歩道の形状が異なる場合
                    // 中心線の方向確認
                    CVector2D npt1(
                        neighbor.second->m_pedestrianCrossingData.m_centerLineData.centerLine.front().x(),
                        neighbor.second->m_pedestrianCrossingData.m_centerLineData.centerLine.front().y());
                    CVector2D npt2(
                        neighbor.second->m_pedestrianCrossingData.m_centerLineData.centerLine.back().x(),
                        neighbor.second->m_pedestrianCrossingData.m_centerLineData.centerLine.back().y());
                    CVector2D nvec2d = npt2 - npt1;
                    double dAngle = CGeoUtil::Angle(vec2d, nvec2d);
                    if (CEpsUtil::Greater(dAngle, 90.0))
                        dAngle = 180 - dAngle;
                    if (CEpsUtil::LessEqual(dAngle, 3.0))
                    {
                        // 中心線の方向が同一
                        BoostPolygon tmpPolygon1 = CBoostGeoUtil::Conv(
                            crossing->m_pedestrianCrossingData.m_mbr);
                        BoostPolygon tmpPolygon2 = CBoostGeoUtil::Conv(
                            neighbor.second->m_pedestrianCrossingData.m_mbr);
                        double dDist = bg::distance(tmpPolygon1, tmpPolygon2);

                        if (CEpsUtil::LessEqual(dDist, dDistTh))
                        {
                            // 距離が近い場合、縦線タイプの横断歩道を不使用にする
                            if (!crossing->m_pedestrianCrossingData.m_bStripes)
                            {
                                crossing->m_pedestrianCrossingData.m_bUse = false;
                            }
                            else
                            {
                                neighbor.second->m_pedestrianCrossingData.m_bUse = false;
                            }
                        }
                    }
                }
            }
        }
    }
}

// 横断歩道の中心線作成の前処理
void CFurnitureDataUtil::preprocessForPedestrianCrossing(
    const Boost3DHashMultiPolygon &srcPolygons,
    BoostMultiPolygon &simplePolygons,
    BoostMultiPolygon &validPolygons,
    Boost3DHashMultiPolygon &mbrs,
    CRotateAngleVecDataManager &rotMngForCheckType,
    CRotateAngleVecDataManager &rotMngForStripe,
    const double dLengthTh,
    const double dAreaRateTh)
{
    const CVector2D vecEast(1, 0);

    for (const auto &srcPoly : srcPolygons)
    {
        // ジオメトリごとにMBR
        double dShort, dLong;
        BoostPolygon srcPolygon, simplePolygon;
        for (const auto &pt : srcPoly.outer())
            srcPolygon.outer().push_back(BoostPoint(pt.x(), pt.y()));
        bg::simplify(srcPolygon, simplePolygon, 0.1);
        simplePolygons.push_back(simplePolygon);
        Boost3DHashPolygon mbr;
        if (CBoostGeoUtil::MBR(simplePolygon, mbr, dShort, dLong))
        {
            // 面積比率確認
            BoostPolygon poly;
            for (const auto &pt : mbr.outer())
                poly.outer().push_back(BoostPoint(pt.x(), pt.y()));
            double dSrcArea = bg::area(simplePolygon);
            double dMbrArea = bg::area(poly);
            double dAreaRate = dSrcArea / dMbrArea;
            if (CEpsUtil::GreaterEqual(dLong, dLengthTh) && CEpsUtil::GreaterEqual(dAreaRate, dAreaRateTh))
            {
                validPolygons.push_back(simplePolygon);
                mbrs.push_back(mbr);

                // 長辺の方向確認
                double dLongLength = 0;
                double dLongEdgeAngle;
                CVector2D longEdge;
                CVector2D longEdgeStartPt;
                for (size_t t = 0; t < simplePolygon.outer().size() - 1; t++)
                {
                    CVector2D pt1(simplePolygon.outer().at(t).x(), simplePolygon.outer().at(t).y());
                    CVector2D pt2(simplePolygon.outer().at(t + 1).x(), simplePolygon.outer().at(t + 1).y());
                    CVector2D tmpVec = pt2 - pt1;
                    double dLength = tmpVec.Length();
                    double dAngle = CGeoUtil::SignedAngle(vecEast, tmpVec);
                    if (CEpsUtil::Greater(dAngle, 90.0))
                    {
                        tmpVec *= -1;
                        dAngle -= 180.0;
                    }
                    if (CEpsUtil::Less(dAngle, -90.0))
                    {
                        tmpVec *= -1;
                        dAngle += 180.0;
                    }
                    tmpVec.Normalize();
                    // 縞タイプの回転方向決定用のデータ作成
                    CRotateAngleVecData d(tmpVec, pt1, dAngle, dLength, 1);
                    rotMngForStripe.Add(d);

                    if (CEpsUtil::Greater(dLength, dLongLength))
                    {
                        dLongLength = dLength;
                        dLongEdgeAngle = dAngle;
                        longEdge = tmpVec;
                        longEdgeStartPt = pt1;
                    }
                }

                // 横断歩道の種別判定用のデータ作成
                CRotateAngleVecData d(longEdge, longEdgeStartPt, dLongEdgeAngle, dLongLength, 1);
                rotMngForCheckType.Add(d);
            }
        }
    }

    // 頻出数でソート
    rotMngForStripe.Sort();
    rotMngForCheckType.Sort();
}

/*!
 平均、分散、標準偏差の算出
*/
bool CFurnitureDataUtil::calcParam(
    const std::vector<double> values,
    double &dAve, double &dVar, double &dStd)
{
    dAve = 0;
    dVar = 0;
    dStd = 0;
    bool bRet = false;
    if (values.size() > 0)
    {
        double dNum = static_cast<double>(values.size());

        // 平均
        double dTotal = 0;
        for (const double &v : values)
            dTotal += v;

        dAve = dTotal / dNum;

        // 分散
        dTotal = 0;
        for (const double &v : values)
            dTotal += pow(v - dAve, 2);
        dVar = dTotal / dNum;

        // 標準偏差
        dStd = sqrt(dVar);

        bRet = true;
    }

    return bRet;
}


// 縦線タイプの横断歩道判定用の矩形作成
bool CFurnitureDataUtil::makeRects(
    BoostMultiPolygon &dst,
    const BoostMultiPolygon &src,
    const CVector2D &baseVec,
    const CVector2D &centerPt,
    const double dRectWidth)
{
    const CVector2D vecEast(1, 0);
    dst.clear();

    if (src.size() > 0 && baseVec.Length() > 0)
    {
        // データ変換
        std::vector<std::vector<CVector2D>> polyPts;
        BOOST_FOREACH(const auto & polygon, src)
        {
            std::vector<CVector2D> pts;
            for (const auto &pt : polygon.outer())
            {
                pts.push_back(CVector2D(pt.x(), pt.y()));
            }
            polyPts.push_back(pts);
        }

        // 回転角
        double dAngle = CGeoUtil::Angle(vecEast, baseVec);
        // 回転方向
        dAngle = CEpsUtil::Less(baseVec.y, 0) ? dAngle : -dAngle;

        // 回転
        BoostMultiPolygon mbrInputPolygon;
        for (const auto &pts : polyPts)
        {
            std::vector<CVector2D> rotPts = CGeoUtil::Rotate(pts, centerPt, dAngle);
            BoostPolygon tmpPolygon;
            for (const auto &pt : rotPts)
                tmpPolygon.outer().push_back(BoostPoint(pt.x, pt.y));
            mbrInputPolygon.push_back(tmpPolygon);
        }

        // mbr
        BoostBox box;
        bg::envelope(mbrInputPolygon, box);
        // x軸方向に切断
        std::vector<std::vector<CVector2D>> halfRects;
        std::vector<CVector2D> rect1, rect2;
        CVector2D bl = CVector2D(box.min_corner().x(), box.min_corner().y());
        CVector2D tl = CVector2D(box.min_corner().x(), box.max_corner().y());
        CVector2D tr = CVector2D(box.max_corner().x(), box.max_corner().y());
        CVector2D br = CVector2D(box.max_corner().x(), box.min_corner().y());
        CVector2D vec1 = tl - bl;
        CVector2D vec2 = tr - br;
        vec1.Normalize();
        vec2.Normalize();

        rect1.push_back(bl);
        rect1.push_back(br);
        rect1.push_back(vec2 * dRectWidth + br);
        rect1.push_back(vec1 * dRectWidth + bl);
        rect1.push_back(bl);
        halfRects.push_back(rect1);
        rect2.push_back(tl);
        rect2.push_back(vec1 * -dRectWidth + tl);
        rect2.push_back(vec2 * -dRectWidth + tr);
        rect2.push_back(tr);
        rect2.push_back(tl);
        halfRects.push_back(rect2);

        // 逆回転
        for (const auto &rect : halfRects)
        {
            std::vector<CVector2D> invRotPts = CGeoUtil::Rotate(rect, centerPt, -dAngle);
            BoostPolygon poly;
            for (const auto &pt : invRotPts)
            {
                poly.outer().push_back(BoostPoint(pt.x, pt.y));
            }
            if (poly.outer().size() > 0)
                dst.push_back(poly);
        }
    }
    return dst.size() > 1;
}

void CRotateAngleVecDataManager::Add(const CRotateAngleVecData &datum)
{
    // 既存データの確認
    std::vector<CRotateAngleVecData>::iterator it = data.begin();
    std::vector<CRotateAngleVecData>::iterator targetIt = data.end();
    double dDiff = 0;
    for (; it != data.end(); it++)
    {
        if (CEpsUtil::GreaterEqual(datum.dAngle, it->dAngle - dEpsilon)
            && CEpsUtil::LessEqual(datum.dAngle, it->dAngle + dEpsilon))
        {
            if (targetIt == data.end())
            {
                targetIt = it;
                dDiff = abs(it->dAngle - datum.dAngle);
            }
            else
            {
                double dTmpDiff = abs(it->dAngle - datum.dAngle);
                if (CEpsUtil::Less(dTmpDiff, dDiff))
                {
                    targetIt = it;
                    dDiff = dTmpDiff;
                }
            }
        }
    }

    if (targetIt == data.end())
    {
        // 新規追加
        data.push_back(datum);
    }
    else
    {
        // 既存更新
        targetIt->nCount += 1;
        if (CEpsUtil::Less(targetIt->dLength, datum.dLength))
        {
            targetIt->dLength = datum.dLength;    // 最長エッジ長を代表値とする
            targetIt->vec = datum.vec;
            targetIt->pos = datum.pos;
            targetIt->dAngle = datum.dAngle;
        }
    }
}
