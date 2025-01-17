#pragma once
#include <vector>
#include <map>
#include "BoostCommon.h"
#include "Boost3DPointHash.h"
#include "CTranRoadData.h"
#include "CBridgeData.h"
#include "SettingData.h"
#include "boost/geometry/index/rtree.hpp"

#pragma region 道路用
/*!
 * @brief 探索結果クラス
*/
class CSearchOverlapRoadResult
{
public:
    /*!
     * @brief 道路情報ポインタ
    */
    std::shared_ptr<CTranRoadData> tranDataPtr;

    /*!
     * @brief ポリゴンポインタ
    */
    std::shared_ptr<Boost3DHashPolygon> polygonPtr;

    /*!
     * @brief tran:function
    */
    int nFunction;


    /*!
     * @brief コンストラクタ
    */
    CSearchOverlapRoadResult() :nFunction(0) {};

    /*!
     * @brief コンストラクタ
    */
    CSearchOverlapRoadResult(const std::shared_ptr<CTranRoadData> &roadPtr,
        const std::shared_ptr<Boost3DHashPolygon> &srcPtr, const int function)
        : tranDataPtr(roadPtr), polygonPtr(srcPtr), nFunction(function) {};
    /*!
     * @brief デストラクタ
    */
    virtual ~CSearchOverlapRoadResult() {};

    /*!
     * @brief コピーコンストラクタ
    */
    CSearchOverlapRoadResult(const CSearchOverlapRoadResult &x) { *this = x; }

    /*!
     * @brief 代入演算子
    */
    CSearchOverlapRoadResult &operator = (const CSearchOverlapRoadResult &x)
    {
        if (this != &x)
        {
            this->tranDataPtr = x.tranDataPtr;
            this->polygonPtr = x.polygonPtr;
            this->nFunction = x.nFunction;
        }
        return *this;
    }
};

/*!
 * @brief 注目ジオメトリと重畳する道路ポリゴンを探索するクラス
*/
class CSearchOverlapRoads
{
private:

    /*!
     * @brief RTree定義
    */
    typedef bg::index::rtree<
        std::pair<BoostBox, CSearchOverlapRoadResult>, bg::index::quadratic<16>> SearchOverlapRoadsRTree;

    bool m_bSetData;                                // データを設定済みか確認するフラグ
    SearchOverlapRoadsRTree m_rtree;                // 探索用RTree

    /*!
     * @brief 探索用データセット一式の解放
    */
    void releaseData();


public:
    typedef std::map<std::shared_ptr<CTranRoadData>, std::vector<CSearchOverlapRoadResult>> ResultMap;

    /*!
     * @brief コンストラクタ
    */
    CSearchOverlapRoads();

    /*!
     * @brief デストラクタ
    */
    ~CSearchOverlapRoads();

    /*!
     * @brief 道路情報の入力
     * @param tranData 道路情報のポインタ群
     * @param lodType  使用するLOD
    */
    void SetData(
        const std::vector<std::shared_ptr<CTranRoadData>> &tranData,
        const CInputSettingData::LODType &lodType);

    /*!
     * @brief データクリア
    */
    void Clear(void) { releaseData(); };

    /*!
     * @brief 入力ポリゴンに2次元平面上で重畳するポリゴンの取得
     * @param polygon 注目ポリゴン
     * @return 重畳ポリゴンのマップ(key : tran:Roadのポインタ, value : ポリゴンリスト)
    */
    ResultMap Search(const Boost3DHashPolygon &polygon);

    /*!
     * @brief 入力頂点に2次元平面上で重畳するポリゴンの取得
     * @param pt 注目頂点
     * @return 重畳ポリゴンのマップ(key : tran:Roadのポインタ, value : ポリゴンリスト)
    */
    ResultMap Search(const Boost3DPointHash &pt);
};
#pragma endregion 道路用

#pragma region 横断歩道橋用
/*!
 * @brief 探索結果クラス
*/
class CSearchOverlapBridgeResult
{
public:
    /*!
     * @brief 横断歩道橋情報ポインタ
    */
    std::shared_ptr<CBridgeData> bridDataPtr;

    /*!
     * @brief ポリゴンポインタ
    */
    std::shared_ptr<Boost3DHashPolygon> polygonPtr;

    /*!
     * @brief コンストラクタ
    */
    CSearchOverlapBridgeResult() {};

    /*!
     * @brief コンストラクタ
    */
    CSearchOverlapBridgeResult(const std::shared_ptr<CBridgeData> &bridPtr,
        const std::shared_ptr<Boost3DHashPolygon> &srcPtr)
        : bridDataPtr(bridPtr), polygonPtr(srcPtr) {};
    /*!
     * @brief デストラクタ
    */
    virtual ~CSearchOverlapBridgeResult() {};

    /*!
     * @brief コピーコンストラクタ
    */
    CSearchOverlapBridgeResult(const CSearchOverlapBridgeResult &x) { *this = x; }

    /*!
     * @brief 代入演算子
    */
    CSearchOverlapBridgeResult &operator = (const CSearchOverlapBridgeResult &x)
    {
        if (this != &x)
        {
            this->bridDataPtr = x.bridDataPtr;
            this->polygonPtr = x.polygonPtr;
        }
        return *this;
    }
};

/*!
 * @brief 注目ジオメトリと重畳する横断歩道橋ポリゴンを探索するクラス
*/
class CSearchOverlapBridge
{
private:

    /*!
     * @brief RTree定義
    */
    typedef bg::index::rtree<
        std::pair<BoostBox, CSearchOverlapBridgeResult>, bg::index::quadratic<16>> SearchOverlapBridgeTree;

    bool m_bSetData;                                // データを設定済みか確認するフラグ
    SearchOverlapBridgeTree m_rtree;                // 探索用RTree

    /*!
     * @brief 探索用データセット一式の解放
    */
    void releaseData();


public:
    // 重畳ポリゴン探索結果のデータ型
    typedef std::map<std::shared_ptr<CBridgeData>, std::vector<CSearchOverlapBridgeResult>> ResultMap;

    /*!
     * @brief コンストラクタ
    */
    CSearchOverlapBridge();

    /*!
     * @brief デストラクタ
    */
    ~CSearchOverlapBridge();

    /*!
     * @brief 橋梁情報の入力
     * @param bridData 横断歩道橋情報のポインタ群
    */
    void SetData(
        const std::vector<std::shared_ptr<CBridgeData>> &bridData);

    /*!
     * @brief データクリア
    */
    void Clear(void) { releaseData(); };

    /*!
     * @brief 入力頂点に2次元平面上で重畳するポリゴンの取得
     * @param pt 注目頂点
     * @return 重畳ポリゴンのマップ(key : brid:Bridgeのポインタ, value : ポリゴンリスト)
    */
    ResultMap Search(const Boost3DPointHash &pt);
};
#pragma endregion 横断歩道橋用
