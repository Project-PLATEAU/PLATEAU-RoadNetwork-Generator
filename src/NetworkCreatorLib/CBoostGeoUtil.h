#pragma once

#include "Boost3DPointHash.h"
#include "CGeoUtil.h"

/*!
 * @brief Boost Geometry の幾何計算ユーティリティクラス
*/
class CBoostGeoUtil
{
public:
    /*!
     * @brief MBR
     * @param[in]   src         入力ポリゴン
     * @param[out]  dst         MBR
     * @param[out]  dShortEdge  mbrの短辺長
     * @param[out]  dLongEdge   mbrの長辺長
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    static bool MBR(
        const BoostPolygon &src,
        Boost3DHashPolygon &dst,
        double &dShortEdge,
        double &dLongEdge);

    /*!
     * @brief MBR
     * @param[in]   src         入力ポリゴン
     * @param[out]  dst         MBR
     * @param[out]  dShortEdge  mbrの短辺長
     * @param[out]  dLongEdge   mbrの長辺長
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    static bool MBR(
        const Boost3DHashPolygon &src,
        Boost3DHashPolygon &dst,
        double &dShortEdge,
        double &dLongEdge);

    /*!
     * @brief MBR
     * @param[in]   src         入力ポリゴン群
     * @param[in]   baseVec     回転角算出用ベクトル
     * @param[in]   centerPt    回転中心座標
     * @param[out]  dst         MBR
     * @param[out]  dShortEdge  mbrの短辺長
     * @param[out]  dLongEdge   mbrの長辺長
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    static bool MBR(
        const BoostMultiPolygon &src,
        const CVector2D &baseVec,
        const CVector2D &centerPt,
        Boost3DHashPolygon &dst,
        double &dShortEdge,
        double &dLongEdge);

    /*!
     * @brief 入力ポリゴンにバッファを付与する
     * @param[in]   polygon バッファリング対象
     * @param[in]   dDist   バッファ距離(m)
     * @return      ポリゴン
    */
    static BoostPolygon Buffering(
        const BoostPolygon &polygon,
        const double dDist);

    /*!
     * @brief 2Dポリゴンに変換
     * @param[in] polygon 3Dポリゴン
     * @return 2Dポリゴン
    */
    static BoostPolygon Conv(const Boost3DHashPolygon &polygon);

    /*!
     * @brief 2Dポリラインに変換
     * @param[in] polyline 3Dポリライン
     * @return 2Dポリライン
    */
    static BoostPolyline Conv(const Boost3DHashPolyline &polyline);

    /*!
     * @brief 2Dポイントに変換
     * @param[in] pt 3Dポイント
     * @return 2Dポイント
    */
    static BoostPoint Conv(const Boost3DPointHash &pt) { return BoostPoint(pt.x(), pt.y()); };

    /*!
     * @brief CVector2Dに変換
     * @param[in] pt 2Dポイント
     * @return CVector2D
    */
    static CVector2D ToCVector2D(const Boost3DPointHash &pt) { return CVector2D(pt.x(), pt.y()); };

    /*!
     * @brief CVector3Dに変換
     * @param[in] pt 3Dポイント
     * @return CVector3D
    */
    static CVector3D ToCVector3D(const Boost3DPointHash &pt) { return CVector3D(pt.x(), pt.y(), pt.z()); };


    /*!
     * @brief 2D衝突判定
     * @param[in] polygon   ポリゴン
     * @param[in] polyline  ポリライン
     * @return 判定結果
     * @retval true     非衝突
     * @retval false    衝突
    */
    static bool Disjoint(const Boost3DHashPolygon &polygon,
        const Boost3DHashPolyline &polyline);

    /*!
     * @brief 2D衝突判定
     * @param[in] polygon1  ポリゴン1
     * @param[in] polygon2  ポリゴン2
     * @return 判定結果
     * @retval true     非衝突
     * @retval false    衝突
    */
    static bool Disjoint(const Boost3DHashPolygon &polygon1,
        const Boost3DHashPolygon &polygon2);

    /*!
     * @brief 2D衝突判定
     * @param[in] polygon   ポリゴン
     * @param[in] pt        ポイント
     * @return 判定結果
     * @retval true     非衝突
     * @retval false    衝突
    */
    static bool Disjoint(const Boost3DHashPolygon &polygon,
        const Boost3DPointHash &pt);

    /*!
     * @brief 面積算出
     * @param[in] polygon ポリゴン
     * @return    面積
    */
    static double Area(const Boost3DHashPolygon &polygon);

    /*!
     * @brief 線分の中点を取得する
     * @param[in]   startPt   始点
     * @param[in]   endPt     終点
     * @return      中点
    */
    static Boost3DPointHash GetCenterPt(
        const Boost3DPointHash &startPt,
        const Boost3DPointHash &endPt);

    /*!
     * @brief ポリラインのサンプリング
     * @param[in] src       入力ポリライン
     * @param[in] interval  サンプリング間隔
     * @return サンプリング後のポリライン
    */
    static Boost3DHashPolyline Sampling(
        const Boost3DHashPolyline &src,
        const double interval);

    /*!
     * @brief マルチポリラインのサンプリング
     * @param[in] src       入力マルチポリライン
     * @param[in] interval  サンプリング間隔
     * @return サンプリング後のマルチポリライン
    */
    static Boost3DHashMultiLines Sampling(
        const Boost3DHashMultiLines &src, const double interval);

    /*!
     * @brief ポリラインのサンプリング
     * @param[in] src   入力ポリライン
     * @param[in] num   サンプリング個数
     * @return サンプリング後のポリライン
    */
    static Boost3DHashPolyline Sampling(
        const Boost3DHashPolyline &src, const int num);

    /*!
     * @brief マルチポリラインのサンプリング
     * @param[in] src   入力マルチポリライン
     * @param[in] num   サンプリング個数
     * @return サンプリング後のマルチポリライン
    */
    static Boost3DHashMultiLines Sampling(
        const Boost3DHashMultiLines &src, const int num);

    /*!
     * @brief ポリラインの水平距離サンプリング
     * @param[in] src       入力ポリライン
     * @param[in] interval  サンプリング間隔
     * @return サンプリング後のポリライン
     * @note   サンプリング後のポリラインのz座標は0固定
    */
    static Boost3DHashPolyline Sampling2D(
        const Boost3DHashPolyline &src,
        const double interval);

    /*!
     * @brief 頂点から鉛直方向に延びる直線と平面との交点位置の算出
     * @param[in] polygon   平面のパラメータを算出するポリゴン(三角メッシュを想定)
     * @param[in] pt        鉛直方向の直線の始点
     * @param[out] crossPt  交点
     * @return 処理結果
     * @retval true         成功
     * @retval false        失敗
    */
    static bool CalcVerticalCrossPt(
        const Boost3DHashPolygon &polygon,
        const Boost3DPointHash &pt,
        Boost3DPointHash &crossPt);

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
    static bool Gradient(
        const Boost3DPointHash &startPt,
        const Boost3DPointHash &endPt,
        double &dGradient,
        bool &isEndHigher);

private:

};
