#include "CBoostGeoUtil.h"
#include "CEpsUtil.h"
#include "boost/foreach.hpp"
#include <vector>

// MBR
bool CBoostGeoUtil::MBR(const BoostPolygon &src, Boost3DHashPolygon &dst, double &dShortEdge, double &dLongEdge)
{
    const CVector2D vecEast(1, 0);
    dst.clear();
    dShortEdge = 0;
    dLongEdge = 0;
    bool bRet = false;

    if (src.outer().size() > 0)
    {
        // 基準エッジの取得(最長辺を基準辺とする)とデータ変換
        std::vector<CVector2D> pts;
        CVector2D baseVec;
        double dLength = 0;
        CVector2D centerPt;
        for (size_t t = 0; t < src.outer().size() - 1; t++)
        {
            CVector2D pt1(src.outer().at(t).x(), src.outer().at(t).y());
            CVector2D pt2(src.outer().at(t + 1).x(), src.outer().at(t + 1).y());
            CVector2D vec = pt2 - pt1;

            if (CEpsUtil::Less(dLength, vec.Length()))
            {
                dLength = vec.Length();
                baseVec = vec;
                centerPt = pt1;
            }
            pts.push_back(pt1);
        }
        pts.push_back(CVector2D(src.outer().back().x(), src.outer().back().y()));   // 終点追加
        baseVec.Normalize();

        // 回転角
        double dAngle = CGeoUtil::Angle(vecEast, baseVec);
        // 回転方向
        dAngle = CEpsUtil::Less(baseVec.y, 0) ? dAngle : -dAngle;

        // 回転
        std::vector<CVector2D> rotPts = CGeoUtil::Rotate(pts, centerPt, dAngle);
        // mbr
        BoostPolygon tmpPolygon;
        for (const auto &pt : rotPts)
            tmpPolygon.outer().push_back(BoostPoint(pt.x, pt.y));
        BoostBox box;
        bg::envelope(tmpPolygon, box);
        std::vector<CVector2D> mbrPts;  // 反時計回りが表
        mbrPts.push_back(CVector2D(box.min_corner().x(), box.min_corner().y()));
        mbrPts.push_back(CVector2D(box.max_corner().x(), box.min_corner().y()));
        mbrPts.push_back(CVector2D(box.max_corner().x(), box.max_corner().y()));
        mbrPts.push_back(CVector2D(box.min_corner().x(), box.max_corner().y()));
        mbrPts.push_back(CVector2D(box.min_corner().x(), box.min_corner().y()));
        // 逆回転
        std::vector<CVector2D> invMbrPts = CGeoUtil::Rotate(mbrPts, centerPt, -dAngle);
        for (const auto &pt : invMbrPts)
        {
            dst.outer().push_back(Boost3DPointHash(pt.x, pt.y, 0));
        }
        double dWidth = box.max_corner().x() - box.min_corner().x();
        double dHeight = box.max_corner().y() - box.min_corner().y();
        dShortEdge = (dWidth < dHeight) ? dWidth : dHeight;
        dLongEdge = (dWidth < dHeight) ? dHeight : dWidth;

        bRet = (dst.outer().size() > 0) ? true : false;
    }
    return bRet;
}

// MBR
bool CBoostGeoUtil::MBR(const Boost3DHashPolygon &src, Boost3DHashPolygon &dst, double &dShortEdge, double &dLongEdge)
{
    BoostPolygon srcPolygon;
    for (const auto &pt : src.outer())
        srcPolygon.outer().push_back(BoostPoint(pt.x(), pt.y()));

    return MBR(srcPolygon, dst, dShortEdge, dLongEdge);
}

// MBR
bool CBoostGeoUtil::MBR(
    const BoostMultiPolygon &src,
    const CVector2D &baseVec,
    const CVector2D &centerPt,
    Boost3DHashPolygon &dst,
    double &dShortEdge,
    double &dLongEdge)
{
    const CVector2D vecEast(1, 0);
    dst.clear();
    dShortEdge = 0;
    dLongEdge = 0;
    bool bRet = false;

    if (src.size() > 0 && baseVec.Length() > 0)
    {
        // データ変換
        std::vector<std::vector<CVector2D>> polyPts;
        BOOST_FOREACH(const auto &polygon, src)
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
        std::vector<CVector2D> mbrPts;
        mbrPts.push_back(CVector2D(box.min_corner().x(), box.min_corner().y()));
        mbrPts.push_back(CVector2D(box.min_corner().x(), box.max_corner().y()));
        mbrPts.push_back(CVector2D(box.max_corner().x(), box.max_corner().y()));
        mbrPts.push_back(CVector2D(box.max_corner().x(), box.min_corner().y()));
        mbrPts.push_back(CVector2D(box.min_corner().x(), box.min_corner().y()));
        // 逆回転
        std::vector<CVector2D> invMbrPts = CGeoUtil::Rotate(mbrPts, centerPt, -dAngle);
        for (const auto &pt : invMbrPts)
        {
            dst.outer().push_back(Boost3DPointHash(pt.x, pt.y, 0));
        }
        double dWidth = box.max_corner().x() - box.min_corner().x();
        double dHeight = box.max_corner().y() - box.min_corner().y();
        dShortEdge = (dWidth < dHeight) ? dWidth : dHeight;
        dLongEdge = (dWidth < dHeight) ? dHeight : dWidth;

        bRet = (dst.outer().size() > 0) ? true : false;
    }
    return bRet;
}

// 入力ポリゴンにバッファを付与する
BoostPolygon CBoostGeoUtil::Buffering(
    const BoostPolygon &polygon,
    const double dDist)
{
    // バッファリングストラテジー
    bg::strategy::buffer::distance_symmetric<double> distStrategy(dDist);
    bg::strategy::buffer::join_miter joinStrategy;
    bg::strategy::buffer::end_flat endStrategy;
    bg::strategy::buffer::point_circle pointStrategy;
    bg::strategy::buffer::side_straight sideStrategy;

    BoostMultiPolygon dstPolygon;
    bg::buffer(polygon, dstPolygon, distStrategy, sideStrategy,
        joinStrategy, endStrategy, pointStrategy);

    return dstPolygon.front();
}

/*!
 * @brief 3Dポリゴンを2Dポリゴンに変換(z座標を落とす)
 * @param[in]   polygon ポリゴン
 * @return Boostのポリゴン
*/
BoostPolygon CBoostGeoUtil::Conv(const Boost3DHashPolygon &polygon)
{
    BoostPolygon dst;
    for (const auto &pt : polygon.outer())
    {
        dst.outer().push_back(BoostPoint(pt.x(), pt.y()));
    }

    for (const auto &ring : polygon.inners())
    {
        dst.inners().push_back(BoostPolygon::ring_type());
        for (const auto &pt : ring)
        {
            dst.inners().back().push_back(BoostPoint(pt.x(), pt.y()));
        }
    }
    return dst;
}

/*!
 * @brief 2Dポリラインに変換
 * @param[in] polyline 3Dポリライン
 * @return 2Dポリライン
*/
BoostPolyline CBoostGeoUtil::Conv(const Boost3DHashPolyline &polyline)
{
    BoostPolyline dst;
    for (const auto &pt : polyline)
    {
        dst.push_back(BoostPoint(pt.x(), pt.y()));
    }

    return dst;
}


/*!
 * @brief 2D衝突判定
 * @param[i] polygon    ポリゴン
 * @param[i] polyline   ポリライン
 * @return 判定結果
 * @retval true     非衝突
 * @retval false    衝突
*/
bool CBoostGeoUtil::Disjoint(
    const Boost3DHashPolygon &polygon,
    const Boost3DHashPolyline &polyline)
{
    BoostPolygon tmpPolygon = CBoostGeoUtil::Conv(polygon);
    BoostPolyline tmpLine = CBoostGeoUtil::Conv(polyline);
    return bg::disjoint(tmpLine, tmpPolygon);
}

/*!
 * @brief 2D衝突判定
 * @param[i] polygon1   ポリゴン1
 * @param[i] polygon2   ポリゴン2
 * @return 判定結果
 * @retval true     非衝突
 * @retval false    衝突
*/
bool CBoostGeoUtil::Disjoint(const Boost3DHashPolygon &polygon1,
    const Boost3DHashPolygon &polygon2)
{
    BoostPolygon tmpPolygon1 = CBoostGeoUtil::Conv(polygon1);
    BoostPolygon tmpPolygon2 = CBoostGeoUtil::Conv(polygon2);
    return bg::disjoint(tmpPolygon1, tmpPolygon2);
}

/*!
 * @brief 2D衝突判定
 * @param[in] polygon   ポリゴン
 * @param[in] pt        ポイント
 * @return 判定結果
 * @retval true     非衝突
 * @retval false    衝突
*/
bool CBoostGeoUtil::Disjoint(const Boost3DHashPolygon &polygon,
    const Boost3DPointHash &pt)
{
    BoostPolygon tmpPolygon = CBoostGeoUtil::Conv(polygon);
    BoostPoint tmpPt = CBoostGeoUtil::Conv(pt);
    return bg::disjoint(tmpPolygon, tmpPt);
}

/*!
 * @brief 面積算出
 * @param[in] polygon ポリゴン
 * @return    面積
*/
double CBoostGeoUtil::Area(const Boost3DHashPolygon &polygon)
{
    BoostPolygon tmpPolygon = CBoostGeoUtil::Conv(polygon);
    return bg::area(tmpPolygon);
}

/*!
 * @brief 線分の中点を取得する
 * @param[in]   startPt   始点
 * @param[in]   endPt     終点
 * @return      中点
*/
Boost3DPointHash CBoostGeoUtil::GetCenterPt(
    const Boost3DPointHash &startPt,
    const Boost3DPointHash &endPt)
{
    CVector3D pt1(startPt.x(), startPt.y(), startPt.z());
    CVector3D pt2(endPt.x(), endPt.y(), endPt.z());
    CVector3D vec = pt2 - pt1;
    CVector3D tmpPt = vec * 0.5 + pt1;
    return Boost3DPointHash(tmpPt.x, tmpPt.y, tmpPt.z);
}

/*!
 * @brief ポリラインのサンプリング
 * @param[in] src       入力ポリライン
 * @param[in] interval  サンプリング間隔
 * @return サンプリング後のポリライン
*/
Boost3DHashPolyline CBoostGeoUtil::Sampling(
    const Boost3DHashPolyline &src, const double interval)
{
    if (bg::is_empty(src)
        || CEpsUtil::Zero(interval))
    {
        return src;
    }

    Boost3DHashPolyline sampledPolyline;

    for (int i = 0; i < src.size() - 1; i++)
    {
        // 直線の始点と終点を取得
        Boost3DPointHash firstPoint = src[i];
        Boost3DPointHash secondPoint = src[i + 1];

        // 2点間の距離を計算
        double dx = secondPoint.x() - firstPoint.x();
        double dy = secondPoint.y() - firstPoint.y();
        double dz = secondPoint.z() - firstPoint.z();
        double segmentLength = bg::distance(firstPoint, secondPoint);

        if (CEpsUtil::Less(segmentLength, interval))
        {
            sampledPolyline.emplace_back(firstPoint);
            continue;
        }

        double t = 0.0;
        int j = 0;
        while (true)
        {
            t = (interval * j) / segmentLength;
            if (CEpsUtil::GreaterEqual(t, 1.0))
            {
                break;
            }

            Boost3DPointHash sampledPoint = Boost3DPointHash(firstPoint.x() + t * dx,
                firstPoint.y() + t * dy,
                firstPoint.z() + t * dz);

            sampledPolyline.emplace_back(sampledPoint);

            j++;
        }
    }

    // ポリラインの最終点を追加
    sampledPolyline.emplace_back(src.back());

    return sampledPolyline;
}

/*!
 * @brief マルチポリラインのサンプリング
 * @param src           入力マルチポリライン
 * @param interval      サンプリング間隔
 * @return サンプリング後のマルチポリライン
*/
Boost3DHashMultiLines CBoostGeoUtil::Sampling(
    const Boost3DHashMultiLines &src, const double interval)
{
    Boost3DHashMultiLines sampledPolylineList;

    for (Boost3DHashPolyline polyline : src)
    {
        Boost3DHashPolyline sampledPolyline = Sampling(polyline, interval);
        sampledPolylineList.emplace_back(sampledPolyline);
    }

    return sampledPolylineList;
}

/*!
 * @brief ポリラインのサンプリング
 * @param src       入力ポリライン
 * @param num       サンプリング個数
 * @return サンプリング後のポリライン
*/
Boost3DHashPolyline CBoostGeoUtil::Sampling(
    const Boost3DHashPolyline &src, const int num)
{
    Boost3DHashPolyline sampledPolyline;

    // 入力ポリラインの全体の長さを測定
    double srcPolylineLength = bg::length(src);
    double interval = srcPolylineLength / num;

    return Sampling(src, interval);
}

/*!
 * @brief マルチポリラインのサンプリング
 * @param src       入力マルチポリライン
 * @param num       サンプリング個数
 * @return サンプリング後のマルチポリライン
*/
Boost3DHashMultiLines CBoostGeoUtil::Sampling(
    const Boost3DHashMultiLines &src, const int num)
{
    Boost3DHashMultiLines sampledPolylineList;

    for (Boost3DHashPolyline polyline : src)
    {
        Boost3DHashPolyline sampledPolyline = Sampling(polyline, num);
        sampledPolylineList.emplace_back(sampledPolyline);
    }

    return sampledPolylineList;
}

/*!
 * @brief ポリラインの水平距離サンプリング
 * @param[in] src       入力ポリライン
 * @param[in] interval  サンプリング間隔
 * @return サンプリング後のポリライン
 * @note   サンプリング後のポリラインのz座標は0固定
*/
Boost3DHashPolyline CBoostGeoUtil::Sampling2D(
    const Boost3DHashPolyline &src,
    const double interval)
{
    if (bg::is_empty(src)
        || CEpsUtil::Zero(interval))
    {
        return src;
    }

    Boost3DHashPolyline sampledPolyline;

    for (auto it = src.cbegin(); it < src.cend() - 1; it++)
    {
        // 直線の始点と終点を取得
        CVector2D firstPoint = ToCVector2D(*it);
        CVector2D secondPoint = ToCVector2D(*(it + 1));

        // 2点間の距離を計算
        CVector2D vec = secondPoint - firstPoint;
        double segmentLength = vec.Length();
        vec.Normalize();
        for (double t = 0; CEpsUtil::Less(t, segmentLength); t += interval)
        {
            CVector2D pos = vec * t + firstPoint;
            sampledPolyline.emplace_back(Boost3DPointHash(pos.x, pos.y, 0));
        }
    }

    // ポリラインの最終点を追加
    sampledPolyline.emplace_back(Boost3DPointHash(src.back().x(), src.back().y(), 0));

    return sampledPolyline;
}

/*!
 * @brief 頂点から鉛直方向に延びる直線と平面との交点位置の算出
 * @param[in] polygon   平面のパラメータを算出するポリゴン(三角メッシュを想定)
 * @param[in] pt        鉛直方向の直線の始点
 * @param[out] crossPt  交点
 * @return 処理結果
 * @retval true         成功
 * @retval false        失敗
*/
bool CBoostGeoUtil::CalcVerticalCrossPt(const Boost3DHashPolygon &polygon, const Boost3DPointHash &pt, Boost3DPointHash &crossPt)
{
    const CVector3D vecZ(0, 0, 1.0);
    bool bRet = false;
    if (polygon.outer().size() > 2)
    {
        CPointBase p1(polygon.outer()[0].x(), polygon.outer()[0].y(), polygon.outer()[0].z());
        CPointBase p2(polygon.outer()[1].x(), polygon.outer()[1].y(), polygon.outer()[1].z());
        CPointBase p3(polygon.outer()[2].x(), polygon.outer()[2].y(), polygon.outer()[2].z());
        CPlane plane;
        CGeoUtil::GetPlaneParameter(p1, p2, p3, plane);
        CVector3D vecN = plane.NormalVec();
        double dAngle = CGeoUtil::Angle(vecN, vecZ);

        if (!CEpsUtil::Equal(dAngle, 90.0)) // 垂直面は除外する
        {
            CVector3D tmpCross;
            if (plane.GetInterseectionPt(CBoostGeoUtil::ToCVector3D(pt), vecZ, tmpCross))
            {
                crossPt.x(tmpCross.x);
                crossPt.y(tmpCross.y);
                crossPt.z(tmpCross.z);
                bRet = true;
            }
        }
    }
    return bRet;
}

/*!
 * @brief 勾配
 * @param[in]   startPt     始点
 * @param[in]   endPt       終点
 * @param[out]  dGradient   勾配(0 - 1.0)
 * @param[out]  isEndHigher 始点より終点が高いか否か
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CBoostGeoUtil::Gradient(
    const Boost3DPointHash &startPt,
    const Boost3DPointHash &endPt,
    double &dGradient,
    bool &isEndHigher)
{
    bool bRet = false;
    double dVerticalDist = abs(endPt.z() - startPt.z());
    CVector2D pt1 = ToCVector2D(startPt);
    CVector2D pt2 = ToCVector2D(endPt);
    double dHorizontalDist = (pt2 - pt1).Length();
    isEndHigher = CEpsUtil::Greater(endPt.z(), startPt.z());

    if (!CEpsUtil::Zero(dHorizontalDist))
    {
        dGradient = dVerticalDist / dHorizontalDist;
        bRet = true;
    }
    return bRet;
}
