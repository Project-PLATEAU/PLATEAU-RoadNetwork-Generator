#pragma once
#include "CityGMLCommon.h"
#include "Boost3DPointHash.h"
#include "CBridgeData.h"

/*!
 * @brief 橋梁データのユーティリティクラス
*/
class CBridgeDataUtil
{
public:
    /*!
     * @brief 橋梁品質情報のlodType(LOD2の場合の詳細度)を取得する
     * @param pBridge   CityGMLの橋梁データのポインタ
     * @param dLodType  LOD2の場合の詳細度
     * @return      データの有無
     * @retval      true    有り
     * @retval      false   無し
    */
    static bool GetBridDataQualityAttributeLodType(
        const citygml::CityObject *pBridge, double &dLodType);

    /*!
     * @brief CityObjectからbrid:functionを取得
     * @param pRoad         CityGMLの橋梁データのポインタ
     * @param bridgeData    橋梁データクラス
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
     */
    static bool GetBridFunction(
        const citygml::CityObject *pBridge,
        CBridgeData &bridgeData);

    /*!
     * @brief 歩道橋の場合CityObjectから橋梁データを取得する
     * @param pBridge       CityGMLの橋梁データのポインタ
     * @param bridgeData    橋梁データクラス
     * @param nJPZone       平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    static bool GetPedestrianCrossingBridge(
        const citygml::CityObject *pBridge,
        CBridgeData &bridgeData,
        const int nJPZone);

private:
    /*!
     * @brief LOD1橋梁データの取得
     * @param pBridge       CityGMLの橋梁データのポインタ
     * @param bridgeData    橋梁データクラス
     * @param nJPZone       平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    static bool getBridgeDataLOD1(
        const citygml::CityObject *pBridge,
        CBridgeData &bridgeData,
        const int nJPZone);

    /*!
     * @brief LOD2橋梁データの取得
     * @param pBridge       CityGMLの橋梁データのポインタ
     * @param bridgeData    橋梁データクラス
     * @param nJPZone       平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    static bool getBridgeDataLOD2(
        const citygml::CityObject *pBridge,
        CBridgeData &bridgeData,
        const int nJPZone);
};

