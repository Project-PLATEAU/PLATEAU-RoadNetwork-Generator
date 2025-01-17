#pragma once
#include <vector>
#include <unordered_map>
#include "BoostCommon.h"
#include "Boost3DPointHash.h"
#include "CTranRoadData.h"

/*!
 * @brief Boost3DPointHashMapの値のデータ
*/
typedef std::map<std::shared_ptr<CTranRoadData>, double> Boost3DPointHashMapValue;

/*!
 * @brief Boost3DPointデータをキーとするハッシュマップ
*/
typedef std::unordered_map<
    Boost3DPointHash, Boost3DPointHashMapValue,
    Boost3DPointHash::HashFunc, Boost3DPointHash::RoundEqualFunc> Boost3DPointHashMap;

/*!
* @brief Boost3DPointデータをキー、カウント数を値とするハッシュマップ
*/
typedef std::unordered_map<
    Boost3DPointHash, int,
    Boost3DPointHash::HashFunc, Boost3DPointHash::RoundEqualFunc> Boost3DPointHashCntMap;

/*!
 * @brief 道路ポインタをキーとする道路ポリゴンの特徴点間のポリラインマップ
*/
typedef std::map<std::shared_ptr<CTranRoadData>, Boost3DHashMultiLines> SegmentMap;

/*!
 * @brief 近傍探索クラス
*/
class CSearchNeighbor
{
private:
    std::unordered_map<std::shared_ptr<CTranRoadData>,
        std::set<std::shared_ptr<CTranRoadData>>> m_neighborMap; // 隣接ポリゴンマップ
    Boost3DPointHashMap m_hashMap;  // 座標点をキーとした道路ポインタリストのハッシュマップ
    bool m_bSetData;                // データを設定済みか確認するフラグ
    SegmentMap m_segmentMap;        // 道路ポインタをキーとする道路ポリゴンの特徴点間のポリラインマップ
    int m_iLod;                     // LOD
    double m_dLodType;              // LODの詳細区分

    /*!
     * @brief 探索用データセット一式のメモリ解放
    */
    void releaseData();

    /*!
     * @brief ハッシュマップの更新
     * @param[in]       ring      リングデータ
     * @param[in]       roadPtr   道路ポインタ
     * @param[in/out]   map      ハッシュマップ
    */
    void updateMap(
        const Boost3DHashRing &ring,
        const std::shared_ptr<CTranRoadData> &roadPtr,
        Boost3DPointHashMap &map);

    /*!
     * @brief 近傍道路マップの更新
     * @param[in]       ring      リングデータ
     * @param[in]       roadPtr   道路ポインタ
     * @param[in]       hashMap   ハッシュマップ
     * @param[in/out]   map       近傍道路ハッシュマップ
    */
    void updateNearlyRoadMap(
        const Boost3DHashRing &ring,
        const std::shared_ptr<CTranRoadData> &roadPtr,
        const Boost3DPointHashMap &hashMap,
        std::unordered_map<std::shared_ptr<CTranRoadData>,
            std::set<std::shared_ptr<CTranRoadData>>> &map);

    /*!
     * @brief 隣接道路マップの更新
     * @param[in]       targetPtr   注目道路ポインタ
     * @param[in]       hashMap     ハッシュマップ
     * @param[in]       nearyMap    近傍道路マップ
     * @param[in]       segmentMap  道路セグメントマップ
     * @param[in/out]   map         隣接道路ハッシュマップ
    */
    void updateNeighborRoadMap(
        const std::shared_ptr<CTranRoadData> &targetPtr,
        const std::shared_ptr<CTranRoadData> &otherPtr,
        const Boost3DPointHashMap &hashMap,
        const SegmentMap &segmentMap,
        std::unordered_map<std::shared_ptr<CTranRoadData>,
            std::set<std::shared_ptr<CTranRoadData>>> &map);

    /*!
     * @brief 道路外周線を特徴点で分割してセグメントを作成しセグメントマップを更新する
     * @param[in]       ring        注目道路のリングデータ
     * @param[in]       targetPtr   注目道路ポインタ
     * @param[in]       hashMap     ハッシュマップ
     * @param[in]       map         近傍道路ハッシュマップ
     * @param[in/out]   segmentMap  セグメントハッシュマップ
    */
    void updateSegmentMap(
        const Boost3DHashRing &ring,
        const std::shared_ptr<CTranRoadData> &targetPtr,
        const Boost3DPointHashMap &hashMap,
        const std::unordered_map<std::shared_ptr<CTranRoadData>,
            std::set<std::shared_ptr<CTranRoadData>>> &map,
        SegmentMap &segmentMap);

    /*!
     * @brief ハッシュマップから指定道路の頂点角度を取得する
     * @param[in]   pt          頂点
     * @param[in]   targetPtr   注目道路ポインタ
     * @param[in]   hashMap     ハッシュマップ
     * @param[out]  dAngle      角度degg
     * @return      探索結果
     * @retval      true        発見
     * @retval      false       未発見
    */
    bool getPtAngle(
        const Boost3DPointHash &pt,
        const std::shared_ptr<CTranRoadData> &targetPtr,
        const Boost3DPointHashMap &hashMap,
        double &dAngle);

public:
    /*!
     * @brief コンストラクタ
    */
    CSearchNeighbor();

    /*!
     * @brief コンストラクタ
    */
    CSearchNeighbor(int lod, double lodType);

    /*!
     * @brief デストラクタ
    */
    ~CSearchNeighbor();

    /*!
     * @brief 道路情報の入力
     * @param tranData 道路情報のポインタ群
    */
    void SetData(std::vector<std::shared_ptr<CTranRoadData>> &tranData);

    /*!
     * @brief 隣接ポリゴンマップのゲッター
    */
    std::unordered_map<std::shared_ptr<CTranRoadData>,
        std::set<std::shared_ptr<CTranRoadData>>> GetNeighborMap() { return m_neighborMap; }

};
