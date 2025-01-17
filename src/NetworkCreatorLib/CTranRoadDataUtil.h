#pragma once
#include <vector>
#include "CityGMLCommon.h"
#include "Boost3DPointHash.h"
#include "CTranRoadData.h"

/*!
 * @brief 交通(道路)データのユーティリティクラス
*/
class CTranRoadDataUtil
{
public:
    /*!
     * @brief 道路LOD1ポリゴンの取得
     * @param pRoad     CityGMLの道路データのポインタ
     * @param polygon   道路LOD1ポリゴン
     * @param nJPZone   平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    static bool GetLOD1(
        const citygml::CityObject *pRoad,
        Boost3DHashPolygon &polygon,
        const int nJPZone);

    /*!
     * @brief 道路構造データの取得
     * @param pRoad CityGMLの道路データのポインタ
     * @param attr  道路構造データ
     * @return      道路構造データの有無
     * @retval      true    有り
     * @retval      false   無し
    */
    static bool GetRoadStructureAttribute(
        const citygml::CityObject *pRoad, CUroRoadStructureAttribute &attr);

    /*!
     * @brief 道路品質情報のlodType(LOD3の場合の詳細度)を取得する
     * @param pRoad     CityGMLの道路データのポインタ
     * @param dLodType  LOD3の場合の詳細度
     * @return      データの有無
     * @retval      true    有り
     * @retval      false   無し
    */
    static bool GetTranDataQualityAttributeLodType(
        const citygml::CityObject *pRoad, double &dLodType);

    /*!
     * @brief CityObjectから道路データを取得
     * @param pRoad         CityGMLの道路データのポインタ
     * @param tranRoadData  交通CityGMLデータクラス
     * @param nJPZone       平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
     * @param bLod1Result   LOD1取得結果
     * @param bLod2Result   LOD2取得結果
     * @param bLod3Result   LOD3取得結果
    */
    static void GetTranRoadData(
        const citygml::CityObject *pRoad,
        CTranRoadData &tranRoadData,
        const int nJPZone,
        bool &bLod1Result,
        bool &bLod2Result,
        bool &bLod3Result);

    /*!
     * @brief CityObjectからtran:functionを取得
     * @param pRoad         CityGMLの道路データのポインタ
     * @param tranRoadData  交通CityGMLデータクラス
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
     */
    static bool GetTranFunction(
        const citygml::CityObject *pRoad,
        CTranRoadData &tranRoadData);

    /*!
     * @brief CityObjectからgml:nameを取得
     * @param pRoad         CityGMLの道路データのポインタ
     * @param tranRoadData  交通CityGMLデータクラス
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
     */
    static bool GetGmlName(
        const citygml::CityObject *pRoad,
        CTranRoadData &tranRoadData);

    /*!
     * @brief 歩道部と植栽を融合したポリゴンを取得する(LOD3.2以上用)
     * @param tranRoadData 交通CityGMLデータクラス
     * @return マルチポリゴン
    */
    static Boost3DHashMultiPolygon GetFootpathAndPlantsPolygon(const CTranRoadData &tranRoadData);

    /*!
     * @brief 入力歩道中心線に衝突する歩道部ポリゴンの探索
     * @param[in] tranRoadData  交通CityGMLデータクラス
     * @param[in] line          歩道中心線
     * @param[out] polygon      歩道中心線と衝突する歩道ポリゴン
     * @return  探索結果
     * @retval  true    発見
     * @retval  false   未発見
    */
    static bool SearchFootpathPolygon(
        const CTranRoadData &tranRoadData,
        const Boost3DHashPolyline &centerLine,
        Boost3DHashPolygon &polygon);

private:
    /*!
     * @brief CityObjectから道路LOD1のデータを取得
     * @param pRoad         CityGMLの道路データのポインタ
     * @param tranRoadData  交通CityGMLデータクラス
     * @param nJPZone       平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    static bool getTranRoadDataLOD1(
        const citygml::CityObject *pRoad,
        CTranRoadData &tranRoadData,
        const int nJPZone);

    /*!
     * @brief CityObjectから道路LOD2,3のデータを取得
     * @param pRoad         CityGMLの道路データのポインタ
     * @param tranRoadData  交通CityGMLデータクラス
     * @param nJPZone       平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
     * @param bLod2Result   LOD2取得結果
     * @param bLod3Result   LOD3取得結果
    */
    static void getTranRoadDataLOD23(
        const citygml::CityObject *pRoad,
        CTranRoadData &tranRoadData,
        const int nJPZone,
        bool &bLod2Result,
        bool &bLod3Result);
};
