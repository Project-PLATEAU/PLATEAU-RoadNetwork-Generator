#pragma once
#include "boost/graph/adjacency_list.hpp"
#include "Boost3DPointHash.h"
#include "CTranRoadData.h"
#include "CFurnitureData.h"
#include "CBridgeData.h"
#include <unordered_map>
#include <map>
#include "CGISFileExporter.h"
#include "CSearchOverlapRoads.h"
#include <mutex>

#pragma region グラフ定義
struct BoostVertexProperty; // 頂点プロパティ宣言
struct BoostEdgeProperty;   // エッジプロパティ宣言

/*!
 * @brief   無向グラフ定義
 * @note    隣接構造のコンテナ定義(vecS = std::vector)
            頂点集合のコンテナ定義
            有向・無向指定
            頂点プロパティ指定
            エッジプロパティ指定
*/
typedef boost::adjacency_list<
    boost::vecS, boost::vecS, boost::undirectedS,
    BoostVertexProperty, BoostEdgeProperty> BoostUndirectedGraph;

/*!
 * @brief 頂点ディスクリプタ定義
*/
typedef boost::graph_traits<BoostUndirectedGraph>::vertex_descriptor BoostVertexDesc;

/*!
 * @brief エッジディスクリプタ定義
*/
typedef boost::graph_traits<BoostUndirectedGraph>::edge_descriptor BoostEdgeDesc;

/*!
 * @brief 頂点プロパティ定義
*/
struct BoostVertexProperty
{
    BoostVertexDesc desc;   // デスクリプター
    Boost3DPointHash pt;    // 座標
    bool isSearched;        // 探索済みフラグ
    std::set<std::shared_ptr<CTranRoadData>> srcRoads;              // 関連道路ポインタ群
    std::set<std::shared_ptr<CFurnitureData>> srcFurnitures;        // 関連都市設備ポインタ群(横断歩道用)
    std::set<std::shared_ptr<CBridgeData>> srcBridges;              // 関連橋梁ポインタ群(横断歩道供用)
    std::set<std::shared_ptr<CCenterLineData>> srcCenterLines;      // 関連中心線のポインタ群

    /*!
     * @brief コンストラクタ
    */
    BoostVertexProperty()
    {
        desc = 0;
        pt.x(0.0);
        pt.y(0.0);
        pt.z(0.0);
        isSearched = false;
    }

    /*!
     * @brief コンストラクタ
     * @param[in] p         座標
     * @param[in] roadPtr   関連道路ポインタ
     * @param[in] linePtr   関連道路中心線ポインタ
    */
    BoostVertexProperty(
        const Boost3DPointHash &p,
        const std::shared_ptr<CTranRoadData> &roadPtr,
        const std::shared_ptr<CCenterLineData> &linePtr)
    {
        desc = 0;
        pt = p;
        isSearched = false;
        AddRoadPtr(roadPtr);
        AddCenterLinePtr(linePtr);
    }

    /*!
     * @brief コンストラクタ
     * @param[in] p         座標
     * @param[in] frnPtr    関連都市設備ポインタ
     * @param[in] linePtr   関連中心線ポインタ
    */
    BoostVertexProperty(
        const Boost3DPointHash &p,
        const std::shared_ptr<CFurnitureData> &frnPtr,
        const std::shared_ptr<CCenterLineData> &linePtr)
    {
        desc = 0;
        pt = p;
        isSearched = false;
        AddFurniturePtr(frnPtr);
        AddCenterLinePtr(linePtr);
    }

    /*!
     * @brief コンストラクタ
     * @param[in] p         座標
     * @param[in] roadPtr   関連道路ポインタ
     * @param[in] frnPtr    関連都市設備ポインタ
     * @param[in] bridePtr  関連橋梁ポインタ
     * @param[in] linePtr   関連中心線ポインタ
    */
    BoostVertexProperty(
        const Boost3DPointHash &p,
        const std::shared_ptr<CTranRoadData> &roadPtr,
        const std::shared_ptr<CFurnitureData> &frnPtr,
        const std::shared_ptr<CBridgeData> &bridgePtr,
        const std::shared_ptr<CCenterLineData> &linePtr)
    {
        desc = 0;
        pt = p;
        isSearched = false;
        AddRoadPtr(roadPtr);
        AddFurniturePtr(frnPtr);
        AddBridgePtr(bridgePtr);
        AddCenterLinePtr(linePtr);
    }

    /*!
     * @brief コンストラクタ
     * @param[in] p         座標
     * @param[in] bridgePtr 関連橋梁ポインタ
     * @param[in] linePtr   関連中心線ポインタ
    */
    BoostVertexProperty(
        const Boost3DPointHash &p,
        const std::shared_ptr<CBridgeData> &bridgePtr,
        const std::shared_ptr<CCenterLineData> &linePtr)
    {
        desc = 0;
        pt = p;
        isSearched = false;
        AddBridgePtr(bridgePtr);
        AddCenterLinePtr(linePtr);
    }

    /*!
     * @brief コピーコンストラクタ
    */
    BoostVertexProperty(const BoostVertexProperty &p) { *this = p; }

    /*!
     * @brief 代入演算子
    */
    BoostVertexProperty &operator =(const BoostVertexProperty &p)
    {
        if (&p != this)
        {
            desc = p.desc;
            pt = p.pt;
            isSearched = p.isSearched;
            srcRoads = p.srcRoads;
            srcFurnitures = p.srcFurnitures;
            srcBridges = p.srcBridges;
            srcCenterLines = p.srcCenterLines;
        }
        return *this;
    }

    /*!
     * @brief 関連道路の追加
     * @param[in] ptr 関連道路のポインタ
    */
    void AddRoadPtr(const std::shared_ptr<CTranRoadData> &ptr)
    {
        if (ptr != nullptr)
            srcRoads.insert(ptr);
    }

    /*!
     * @brief 関連道路の追加
     * @param[in] vecPtr 関連道路のポインタ群
    */
    void AddRoadPtr(const std::set<std::shared_ptr<CTranRoadData>> &vecPtr)
    {
        for (const auto &ptr : vecPtr)
            AddRoadPtr(ptr);
    }

    /*!
     * @brief 関連道路の削除
     * @param[in] ptr 関連道路のポインタ
    */
    void DeleteRoadPtr(const std::shared_ptr<CTranRoadData> &ptr)
    {
        auto it = srcRoads.find(ptr);
        if (it != srcRoads.end())
            srcRoads.erase(it);
    }

    /*!
     * @brief 関連道路の一括削除
    */
    void ClearRoadPtr() { srcRoads.clear(); }

    /*!
     * @brief 関連都市設備の追加
     * @param[in] ptr 関連都市設備のポインタ
    */
    void AddFurniturePtr(const std::shared_ptr<CFurnitureData> &ptr)
    {
        if (ptr != nullptr)
            srcFurnitures.insert(ptr);
    }

    /*!
     * @brief 関連都市設備の追加
     * @param[in] vecPtr 関連道路のポインタ群
    */
    void AddFurniturePtr(const std::set<std::shared_ptr<CFurnitureData>> &vecPtr)
    {
        for (const auto &ptr : vecPtr)
            AddFurniturePtr(ptr);
    }

    /*!
     * @brief 関連都市設備の削除
     * @param[in] ptr 関連都市設備のポインタ
    */
    void DeleteFurniturePtr(const std::shared_ptr<CFurnitureData> &ptr)
    {
        auto it = srcFurnitures.find(ptr);
        if (it != srcFurnitures.end())
            srcFurnitures.erase(it);
    }

    /*!
     * @brief 関連都市設備の一括削除
    */
    void ClearFurniturePtr() { srcFurnitures.clear(); }

    /*!
     * @brief 関連橋梁の追加
     * @param[in] ptr 関連橋梁のポインタ
    */
    void AddBridgePtr(const std::shared_ptr<CBridgeData> &ptr)
    {
        if (ptr != nullptr)
            srcBridges.insert(ptr);
    }

    /*!
     * @brief 関連橋梁の追加
     * @param[in] vecPtr 関連橋梁のポインタ群
    */
    void AddBridgePtr(const std::set<std::shared_ptr<CBridgeData>> &vecPtr)
    {
        for (const auto &ptr : vecPtr)
            AddBridgePtr(ptr);
    }

    /*!
     * @brief 関連橋梁の削除
     * @param[in] ptr 関連道路のポインタ
    */
    void DeleteBridgePtr(const std::shared_ptr<CBridgeData> &ptr)
    {
        auto it = srcBridges.find(ptr);
        if (it != srcBridges.end())
            srcBridges.erase(it);
    }

    /*!
     * @brief 関連橋梁の一括削除
    */
    void ClearBridgePtr() { srcBridges.clear(); }

    /*!
     * @brief 関連道路中心線の追加
     * @param[in] ptr 関連道路中心線のポインタ
    */
    void AddCenterLinePtr(const std::shared_ptr<CCenterLineData> &ptr) { srcCenterLines.insert(ptr); }

    /*!
     * @brief 関連中心線の追加
     * @param[in] vecPtr 関連中心線のポインタ群
    */
    void AddCenterLinePtr(const std::set<std::shared_ptr<CCenterLineData>> &vecPtr)
    {
        for (const auto &ptr : vecPtr)
            AddCenterLinePtr(ptr);
    }

    /*!
     * @brief 関連中心線の削除
     * @param[in] ptr 関連道路中心線のポインタ
    */
    void DeleteCenterLinePtr(const std::shared_ptr<CCenterLineData> &ptr)
    {
        auto it = srcCenterLines.find(ptr);
        if (it != srcCenterLines.end())
            srcCenterLines.erase(it);
    }

    /*!
     * @brief 関連道路中心線の一括削除
    */
    void ClearCenterLinePtr() { srcCenterLines.clear(); }
};

/*!
 * @brief グラフのエッジプロパティ定義
*/
struct BoostEdgeProperty
{
    BoostVertexDesc vertexDesc1;                            // 頂点ディスクリプタ―1
    BoostVertexDesc vertexDesc2;                            // 頂点ディスクリプタ―2
    std::shared_ptr<CTranRoadData> srcRoadPtr;              // 関連道路のポインタ
    std::shared_ptr<CFurnitureData> srcFrnPtr;              // 関連都市設備のポインタ
    std::shared_ptr<CBridgeData> srcBridgePtr;              // 関連橋梁のポインタ
    std::shared_ptr<CCenterLineData> srcCenterLinePtr;      // 関連中心線のポインタ
    double dLength;                                         // エッジの長さ(要手動更新)

    /*!
     * @brief コンストラクタ
    */
    BoostEdgeProperty()
    {
        vertexDesc1 = 0;
        vertexDesc2 = 0;
        srcRoadPtr = nullptr;
        srcFrnPtr = nullptr;
        srcBridgePtr = nullptr;
        srcCenterLinePtr = nullptr;
        dLength = 0;
    }

    /*!
     * @brief コンストラクタ
     * @param[in] desc1         頂点デスクリプター
     * @param[in] desc2         頂点デスクリプター
     * @param[in] roadPtr       関連道路のポインタ
     * @param[in] frnPtr        関連都市設備のポインタ
     * @param[in] bridgePtr     関連橋梁のポインタ
     * @param[in] centerLinePtr 関連中心線のポインタ
     * @param[in] length        エッジの長さ
    */
    BoostEdgeProperty(
        BoostVertexDesc desc1, BoostVertexDesc desc2,
        std::shared_ptr<CTranRoadData> roadPtr = nullptr,
        std::shared_ptr<CFurnitureData> frnPtr = nullptr,
        std::shared_ptr<CBridgeData> bridgePtr = nullptr,
        std::shared_ptr<CCenterLineData> centerLinePtr = nullptr,
        double length = 0)
    {
        vertexDesc1 = desc1;
        vertexDesc2 = desc2;
        srcRoadPtr = roadPtr;
        srcFrnPtr = frnPtr;
        srcBridgePtr = bridgePtr;
        srcCenterLinePtr = srcCenterLinePtr;
        dLength = length;
    }

    /*!
     * @brief コピーコンストラクタ
    */
    BoostEdgeProperty(const BoostEdgeProperty &e) { *this = e; }

    /*!
     * @brief 代入演算子
    */
    BoostEdgeProperty &operator =(const BoostEdgeProperty &e)
    {
        if (&e != this)
        {
            vertexDesc1 = e.vertexDesc1;
            vertexDesc2 = e.vertexDesc2;
            srcRoadPtr = e.srcRoadPtr;
            srcFrnPtr = e.srcFrnPtr;
            srcBridgePtr = e.srcBridgePtr;
            srcCenterLinePtr = e.srcCenterLinePtr;
            dLength = e.dLength;
        }
        return *this;
    }
};

#pragma endregion

#pragma region 出力用ネットワークデータクラス

/*!
 * @brief 視覚障害者誘導用ブロック等の有無種別
*/
enum class BRAILLE_TILE_TYPE
{
    DOES_NOT_EXIST = 1, // 点字ブロック無し
    EXIST,              // 点字ブロック有り
    UNKNOWN = 99        // 不明
};

/*!
 * @brief 経路構造
*/
enum class ROUTE_STRUCTURE_TYPE
{
    SEPARATED_DRIVEWAY_AND_SIDEWALK = 1,    // 車道と歩道の物理的な分離有り
    PEEDESTRIAN_CROSSING = 3,               // 横断歩道
    PEDESTRIAN_BRIDGE = 6,                  // 横断歩道橋
    UNKNOWN = 99,                           // 不明
};

/*!
 * @brief 幅員タイプ(歩道)
*/
enum class WIDTH_TYPE
{
    LESS_THAN_1M = 1,                       // 1m未満
    MORE_THAN_1M_BUT_LESS_THAN_2M,          // 1m以上2m未満
    MORE_THAN_2M_BUT_LESS_THAN_3M,          // 2m以上3m未満
    MORE_THAN_3M,                           // 3m以上
    UNKNOWN = 99,                           // 不明
};

/*!
 * @brief 縦断勾配タイプ
*/
enum class VTCL_SLOPE_TYPE
{
    ZERO = 1,                                                       // 0%
    GREATER_THAN_0_BUT_LESS_THAN_OR_EQUAL_5,                        // 0%より大きい5%以下
    GREATER_THAN_5_BUT_LESS_THAN_OR_EQUAL_8_END_POINT_HIGHER,       // 5%より大きい8%以下(起点より終点が高い)
    GREATER_THAN_5_BUT_LESS_THAN_OR_EQUAL_8_START_POINT_HIGHER,     // 5%より大きい8%以下(起点より終点が低い)
    GREATER_THAN_8_BUT_LESS_THAN_OR_EQUAL_18_END_POINT_HIGHER,      // 8%より大きい18%以下(起点より終点が高い)
    GREATER_THAN_8_BUT_LESS_THAN_OR_EQUAL_18_START_POINT_HIGHER,    // 8%より大きい18%以下(起点より終点が低い)
    GREATER_THAN_18_END_POINT_HIGHER,                               // 18%より大きい(起点より終点が高い)
    GREATER_THAN_18_START_POINT_HIGHER,                             // 18%より大きい(起点より終点が低い)
    UNKNOWN = 99,                                                   // 不明
};

/*!
 * @brief 有効値、無効値判定用
*/
enum class IS_VALID
{
    INVALID_VALUE = 0,  // 無効
    VALID_VALUE,        // 有効
    UNKNOWN = 99,       // 不明
};

/*!
 * @brief ノード情報構造体
*/
struct Node
{
    std::string strId;              // ID
    Boost3DPointHash pt;            // 座標
    std::set<std::string> edges;    // 連結エッジID

    /*!
     * @brief コンストラクタ
    */
    Node()
    {
        strId = "";
    }

    /*!
     * @brief コンストラクタ
     * @param[in] strId ノードID
     * @param[in] pt    座標
     * @param[in] edges ノードが接続するリンクID群
    */
    Node(std::string strId, Boost3DPointHash pt, std::set<std::string> edges)
    {
        this->strId = strId;
        this->pt = pt;
        this->edges = edges;
    }

    /*!
     * @brief コピーコンストラクタ
    */
    Node(const Node &n) { *this = n; }

    /*!
     * @brief 代入演算子
    */
    Node &operator =(const Node &n)
    {
        if (&n != this)
        {
            strId = n.strId;
            pt = n.pt;
            edges = n.edges;
        }
        return *this;
    }
};

/*!
 * @brief リンク情報構造体
*/
struct Link
{
    /* 車歩道共通 */
    std::string             strId;          // ID
    std::string             strStartId;     // 開始ノードID
    std::string             strEndId;       // 終点ノードID
    double                  dLength;        // リンク延長
    std::string             strCreateDate;  // 作成・更新日(YYYY-MM-DD)
    bool                    isRefMinWidth;  // 最小幅員が参考値の場合のフラグ(true : 参考値, 歩道用)
    double                  dMinWidth;      // 最小幅員
    Boost3DPointHash        minWidthPos;    // 最小幅員地点
    VTCL_SLOPE_TYPE         vtclSlope;      // 縦断勾配(コード値, LOD3.0以上)
    int                     nMaxVSlope;     // 最大縦断勾配 %単位(LOD3.0以上)
    Boost3DPointHash        vSlopePos;      // 最大縦断勾配地点(LOD3.0以上)
    int                     nAveVSlope;     // 平均縦断勾配 %単位(LOD3.0以上)
    std::string             strName;        // 通り名称または交差点名称
    Boost3DHashPolyline     geom;           // 幾何形状
    /* 制御用フラグ */
    bool                    isValidWidth;   // 幅員の有効フラグ
    bool                    isValidVSlope;  // 縦断勾配の有効フラグ
    bool                    isValidHSlope;  // 横断勾配の有効フラグ
    /* 車道のみ */
    std::string             strType;        // 道路の区分(一般国道,都道府県道,etc.)
    /* 歩道のみ */
    std::string             strRank;        // ランク区分
    WIDTH_TYPE              width;          // 幅員(コード値)
    BRAILLE_TILE_TYPE       brailTile;      // 点字ブロックの有無(0:無し, 1:有り, 99:不明)
    ROUTE_STRUCTURE_TYPE    rtStruct;       // 経路の構造
    int                     nMaxHSlope;     // 最大横断勾配 %単位(LOD3.2以上)
    Boost3DPointHash        hSlopePos;      // 最大横断勾配地点(LOD3.2以上)

    /*!
     * @brief コンストラクタ
    */
    Link(void)
    {
        strId = "";
        strStartId = "";
        strEndId = "";
        dLength = 0;
        strRank = "";
        strCreateDate = "";
        vtclSlope = VTCL_SLOPE_TYPE::UNKNOWN;
        dMinWidth = 0;
        isRefMinWidth = false;
        nMaxVSlope = 99;
        nAveVSlope = 99;
        strName = "";
        strType = "";
        width = WIDTH_TYPE::UNKNOWN;
        brailTile = BRAILLE_TILE_TYPE::UNKNOWN;
        rtStruct = ROUTE_STRUCTURE_TYPE::UNKNOWN;
        nMaxHSlope = 0;
        isValidWidth = false;
        isValidVSlope = false;
        isValidHSlope = false;
    }

    /*!
     * @brief コンストラクタ(車道)
     * @param strId         リンクID
     * @param geom          幾何形状
     * @param dLength       リンク延長
     * @param isValidWidth  有効幅員フラグ
     * @param dMinWidth     最小幅員m
     * @param minWidthPos   最小幅員地点
     * @param strCreateDate 作成・更新日
     * @param strStartId    開始ノード点
     * @param strEndId      終点ノードID
     * @param strName       通り名称または交差点
     * @param strType       道路の区分
     * @param isValidVSlope 有効縦断勾配フラグ
     * @param vtclSlope     縦断勾配(コード値)
     * @param nMaxVSlope    最大縦断勾配
     * @param vSlopePos     最大縦断勾配地点
     * @param nAveVSlope    平均縦断勾配
    */
    Link(
        std::string strId,
        Boost3DHashPolyline geom,
        double dLength,
        bool isValidWidth,
        double dMinWidth,
        Boost3DPointHash minWidthPos,
        std::string strCreateDate = "",
        std::string strStartId = "",
        std::string strEndId = "",
        std::string strName = "",
        std::string strType = "",
        bool isValidVSlope = false,
        VTCL_SLOPE_TYPE vtclSlope = VTCL_SLOPE_TYPE::UNKNOWN,
        int nMaxVSlope = 0,
        int nAveVSlope = 0) : Link()
    {
        this->strId = strId;
        this->geom = geom;
        this->dLength = dLength;
        this->isValidWidth = isValidWidth;
        this->dMinWidth = dMinWidth;
        this->minWidthPos = minWidthPos;
        this->strCreateDate = strCreateDate;
        this->strStartId = strStartId;
        this->strEndId = strEndId;
        this->strName = strName;
        this->strType = strType;
        this->isValidVSlope = isValidVSlope;
        this->vtclSlope = vtclSlope;
        this->nMaxVSlope = nMaxVSlope;
        this->nAveVSlope = nAveVSlope;
    }

    /*!
     * @brief コンストラクタ(歩道用)
     * @param strId         リンクID
     * @param geom          幾何形状
     * @param dLength       リンク延長
     * @param isValidWidth  有効幅員フラグ
     * @param isRefMinWidth 最小幅員が参考値の場合のフラグ
     * @param dMinWidth     最小幅員m
     * @param minWidthPos   最小幅員地点
     * @param width         幅員(コード値)
     * @param strCreateDate 作成・更新日
     * @param strStartId    開始ノード点
     * @param strEndId      終点ノードID
     * @param strName       通り名称または交差点
     * @param strRank       ランク区分
     * @param brailTile     点字ブロックの有無
     * @param rtStruct      経路の構造
     * @param isValidVSlope 有効縦断勾配フラグ
     * @param vtclSlope     縦断勾配(コード値)
     * @param nMaxVSlope    最大縦断勾配
     * @param vSlopePos     最大縦断勾配地点
     * @param nAveVSlope    平均縦断勾配
     * @param isValidHSlope 有効横断勾配フラグ
     * @param nMaxHSlope    最大横断勾配
     * @param hSlopePos     最大横断勾配地点
    */
    Link(
        std::string strId,
        Boost3DHashPolyline geom,
        double dLength,
        bool isValidWidth,
        bool isRefMinWidth,
        double dMinWidth,
        Boost3DPointHash minWidthPos,
        WIDTH_TYPE width = WIDTH_TYPE::UNKNOWN,
        std::string strCreateDate = "",
        std::string strStartId = "",
        std::string strEndId = "",
        std::string strName = "",
        std::string strRank = "",
        BRAILLE_TILE_TYPE brailTile = BRAILLE_TILE_TYPE::UNKNOWN,
        ROUTE_STRUCTURE_TYPE rtStruct = ROUTE_STRUCTURE_TYPE::UNKNOWN,
        bool isValidVSlope = false,
        VTCL_SLOPE_TYPE vtclSlope = VTCL_SLOPE_TYPE::UNKNOWN,
        int nMaxVSlope = 0,
        Boost3DPointHash vSlopePos = Boost3DPointHash(0, 0, 0),
        int nAveVSlope = 0,
        bool isValidHSlope = false,
        int nMaxHSlope = 0,
        Boost3DPointHash hSlopePos = Boost3DPointHash(0, 0, 0)) : Link()
    {
        this->strId = strId;
        this->geom = geom;
        this->dLength = dLength;
        this->isValidWidth = isValidWidth;
        this->isRefMinWidth = isRefMinWidth;
        this->dMinWidth = dMinWidth;
        this->minWidthPos = minWidthPos;
        this->width = width;
        this->strCreateDate = strCreateDate;
        this->strStartId = strStartId;
        this->strEndId = strEndId;
        this->strName = strName;
        this->strRank = strRank;
        this->brailTile = brailTile;
        this->rtStruct = rtStruct;
        this->isValidVSlope = isValidVSlope;
        this->vtclSlope = vtclSlope;
        this->nMaxVSlope = nMaxVSlope;
        this->vSlopePos = vSlopePos;
        this->nAveVSlope = nAveVSlope;
        this->isValidHSlope = isValidHSlope;
        this->nMaxHSlope = nMaxHSlope;
        this->hSlopePos = hSlopePos;
    }

    /*!
     * @brief コピーコンストラクタ
    */
    Link(const Link &link) { *this = link; }

    /*!
     * @brief 代入演算子
    */
    Link &operator =(const Link &link)
    {
        if (&link != this)
        {
            this->strId = link.strId;
            this->geom = link.geom;
            this->dLength = link.dLength;
            this->isValidWidth = link.isValidWidth;
            this->isRefMinWidth = link.isRefMinWidth;
            this->dMinWidth = link.dMinWidth;
            this->minWidthPos = link.minWidthPos;
            this->width = link.width;
            this->strCreateDate = link.strCreateDate;
            this->strStartId = link.strStartId;
            this->strEndId = link.strEndId;
            this->strName = link.strName;
            this->strType = link.strType;
            this->isValidVSlope = link.isValidVSlope;
            this->vtclSlope = link.vtclSlope;
            this->nMaxVSlope = link.nMaxVSlope;
            this->vSlopePos = link.vSlopePos;
            this->nAveVSlope = link.nAveVSlope;
            this->isValidHSlope = link.isValidHSlope;
            this->nMaxHSlope = link.nMaxHSlope;
            this->hSlopePos = link.hSlopePos;
            this->brailTile = link.brailTile;
            this->rtStruct = link.rtStruct;
            this->strRank = link.strRank;
        }
        return *this;
    }

};
#pragma endregion

#pragma region ネットワーク
/*!
 * @brief ネットワーククラス
*/
class CNetwork
{
public:
    /*!
     * @brief 出力ファイル種別
    */
    enum class OUTPUT_FILE_TYPE
    {
        SHP = 0,    // SHPのみ
        GEOJSON,    // GeoJSONのみ
        BOTH,       // 両方
    };

    /*!
     * @brief ネットワークデータ種別
    */
    enum class NETWORK_DATA_TYPE
    {
        ROADWAY = 0,    // 車道
        FOOTPATH        // 歩道
    };

    /*!
     * @brief 頂点探索用rtreeのデータ型
    */
    typedef std::pair<Boost3DPointHash, BoostVertexDesc> VertexRTreeValue;

    /*!
     * @brief コンストラクタ
    */
    CNetwork(const NETWORK_DATA_TYPE type, const double dLod3Detail = 3.0);

    /*!
     * @brief デストラクタ
    */
    ~CNetwork(void) {};

    /*!
     * @brief ネットワーク追加(複数道路一括)
     * @param[in] vecRoad 道路ポインタ群
    */
    void Add(
        const std::vector<std::shared_ptr<CTranRoadData>> &vecRoad);

    /*!
     * @brief ネットワーク追加(複数都市設備一括)
     * @param[in] vecFurniture 都市設備ポインタ群
    */
    void Add(
        const std::vector<std::shared_ptr<CFurnitureData>> &vecFurniture);

    /*!
     * @brief ネットワーク追加(複数橋梁一括)
     * @param[in] vecBridge 橋梁ポインタ群
    */
    void Add(
        const std::vector<std::shared_ptr<CBridgeData>> &vecBridge);

    /*!
     * @brief 点字ブロック情報の設定
     * @param[in] vecFurniture  点字ブロック(都市設備)ポインタ群
     * @param[in] dInterval     点字ブロックのサンプリング間隔
     * @param[in] dLengthTh     ポリライン化した点字ブロックの長さしきい値
    */
    void SetBrailleTile(
        const std::vector<std::shared_ptr<CFurnitureData>> &vecFurniture,
        const double dInterval = 0.1,
        const double dLengthTh = 1.0);

    /*!
     * @brief 頂点座標の更新
     * @param[in] target    更新対象の座標点
     * @param[in] pt        更新後の座標点情報
     * @return  更新結果
     * @retval  true        更新済み
     * @retval  false       未更新
    */
    bool UpdateVertex(const Boost3DPointHash &target, const Boost3DPointHash &pt);

    /*!
     * @brief 頂点座標の更新
     * @param[in] target    更新対象の頂点ディスクリプタ
     * @param[in] pt        更新後の座標点情報
     * @return  更新結果
     * @retval  true        更新済み
     * @retval  false       未更新
    */
    bool UpdateVertex(const BoostVertexDesc &target, const Boost3DPointHash &pt);

    /*!
     * @brief グラフのクリア
    */
    void Clear();

    /*!
     * @brief 最近傍頂点の探索
     * @param[in]   pt        注目点
     * @param[out]  dDist     注目点と最近傍点の距離
     * @return 最近傍点のディスクリプタ
     * @note 最近傍点が無い場合(rtreeが空)の戻り値は、BoostUndirectedGraph::null_vertex()
    */
    BoostVertexDesc NNSearch(const Boost3DPointHash &pt, double &dDist);

    /*!
     * @brief ネットワークデータのファイル出力
     * @param[in] strShpOutputFolder        SHP出力フォルダパス
     * @param[in] strGeoJsonOutputFolder    GeoJSON出力フォルダパス
     * @param[in] nJPZone                   平面直角座標系の系番号
     * @param[in] isUseZ                    z座標の有無
     * @param[in] fileType                  出力ファイル種別
     * @param[in] strEncoding               文字コード
    */
    void OutputNetworkData(
        const std::string &strShpOutputFolder,
        const std::string &strGeoJsonOutputFolder,
        const int nJPZone,
        const bool isUseZ,
        const OUTPUT_FILE_TYPE fileType,
        const std::string strEncoding="CP932");

    /*!
     * @brief 頂点探索用rtreeにおいて、指定する中心線の頂点であるか判定する
     * @param[in] v         rtreeの頂点データ
     * @param[in] tranPtr   道路ポインタ
     * @param[in] linePtr   中心線ポインタ
     * @return 判定結果
     * @retval  true    中心線の頂点である
     * @retval  false   中心線の頂点ではない
    */
    bool CheckVertex(
        const VertexRTreeValue &v,
        const std::shared_ptr<CTranRoadData> &tranPtr,
        const std::shared_ptr<CCenterLineData> &linePtr)
    {
        return ((m_graph[v.second].srcRoads.find(tranPtr) != m_graph[v.second].srcRoads.end())
            && (m_graph[v.second].srcCenterLines.find(linePtr) != m_graph[v.second].srcCenterLines.end()));
    }

    /*!
     * @brief 頂点探索用rtreeにおいて、指定する中心線の頂点であるか判定する
     * @param[in] v         rtreeの頂点データ
     * @param[in] frnPtr    都市設備ポインタ
     * @param[in] linePtr   中心線ポインタ
     * @return 判定結果
     * @retval  true    中心線の頂点である
     * @retval  false   中心線の頂点ではない
    */
    bool CheckVertex(
        const VertexRTreeValue &v,
        const std::shared_ptr<CFurnitureData> &frnPtr,
        const std::shared_ptr<CCenterLineData> &linePtr)
    {
        return ((m_graph[v.second].srcFurnitures.find(frnPtr) != m_graph[v.second].srcFurnitures.end())
            && (m_graph[v.second].srcCenterLines.find(linePtr) != m_graph[v.second].srcCenterLines.end()));
    }

    /*!
     * @brief 頂点探索用rtreeにおいて、指定する中心線の頂点であるか判定する
     * @param[in] v         rtreeの頂点データ
     * @param[in] bridgePtr 橋梁ポインタ
     * @param[in] linePtr   中心線ポインタ
     * @return 判定結果
     * @retval  true    中心線の頂点である
     * @retval  false   中心線の頂点ではない
    */
    bool CheckVertex(
        const VertexRTreeValue &v,
        const std::shared_ptr<CBridgeData> &bridgePtr,
        const std::shared_ptr<CCenterLineData> &linePtr)
    {
        return ((m_graph[v.second].srcBridges.find(bridgePtr) != m_graph[v.second].srcBridges.end())
            && (m_graph[v.second].srcCenterLines.find(linePtr) != m_graph[v.second].srcCenterLines.end()));
    }

private:
    /*!
     * @brief Boost3DPointHashデータをキーとするハッシュマップ(同一頂点の探索用)
    */
    typedef std::unordered_map<
        Boost3DPointHash, BoostVertexDesc, Boost3DPointHash::HashFunc,
        Boost3DPointHash::RoundEqualFunc> VertexHashMap;

    /*!
     * @brief 関連道路情報の管理用の関連道路のポインタをキー、中心線のポインタ群を値とするハッシュマップ
    */
    typedef std::unordered_map<std::shared_ptr<CTranRoadData>, std::set<std::shared_ptr<CCenterLineData>>> RoadHashMap;
    /*!
     * @brief 関連都市設備情報の管理用の関連都市設備のポインタをキー、中心線のポインタ群を値とするハッシュマップ
    */
    typedef std::unordered_map<std::shared_ptr<CFurnitureData>, std::set<std::shared_ptr<CCenterLineData>>> FurnitureHashMap;

    /*!
     * @brief 関連橋梁情報の管理用の関連橋梁のポインタをキー、中心線のポインタ群を値とするハッシュマップ
    */
    typedef std::unordered_map<std::shared_ptr<CBridgeData>, std::set<std::shared_ptr<CCenterLineData>>> BridgeHashMap;

    /*!
     * @brief 頂点探索用のrtree
    */
    typedef bg::index::rtree<VertexRTreeValue, bg::index::quadratic<16>> VertexRTree;

    /*!
     * @brief 近傍点字ブロックの探索用データ
    */
    typedef std::tuple<Boost3DPointHash,
        std::shared_ptr<CFurnitureData>, std::shared_ptr<Boost3DHashPolygon>> BrailleTileTuple;

    /*!
     * @brief 近傍点字ブロックの探索用RTree定義
    */
    typedef bg::index::rtree<BrailleTileTuple, bg::index::quadratic<16>> SearchBrailleTileRTree;

    NETWORK_DATA_TYPE m_dataType;               // 入出力ネットワークのデータタイプ(車道or歩道判断用)
    BoostUndirectedGraph m_graph;               // 無向グラフ
    VertexHashMap m_vertexMap;                  // グラフ内頂点の探索用ハッシュマップ
    RoadHashMap m_roadHashMap;                  // 関連道路情報の管理用ハッシュマップ
    FurnitureHashMap m_frnHashMap;              // 関連都市設備情報の管理用ハッシュマップ
    BridgeHashMap m_bridgeHashMap;              // 関連橋梁情報の管理用ハッシュマップ
    VertexRTree m_vertexRTree;                  // ノード点探索用
    SearchBrailleTileRTree m_brailleTileRTree;  // 近傍点字ブロック探索用
    const int m_nNodeIdDigit;                   // ノードIDの文字数
    const int m_nLinkIdDigit;                   // リンクIDの文字数
    const int m_nLanLotDigit;                   // 経緯度座標の桁数
    const int m_nLanLotDecimal;                 // 経緯度座標の小数点以下桁数
    double m_dLod3Detail;                       // LOD3の場合の詳細度
    CSearchOverlapRoads m_sor;                  // 注目ジオメトリと重畳する道路ポリゴン探索用
    CSearchOverlapBridge m_sob;                 // 注目ジオメトリと重畳する橋梁ポリゴン探索用
    std::mutex m_nodeIdMutex;                   // 排他制御用
    std::mutex m_nodeDataMutex;                 // 排他制御用
    std::mutex m_linkMutex;                     // 排他制御用
    std::mutex m_linkIdMutex;                   // 排他制御用
    std::mutex m_errLogMutex;                   // 排他制御用

    /*!
     * @brief ネットワーク追加(中心線1本分)
     * @param[in] centerLine    道路中心線のポインタと幅員
     * @param[in] roadPtr       道路のポインタ
    */
    void add(
        const std::shared_ptr<CCenterLineData> &centerLinePtr,
        const std::shared_ptr<CTranRoadData> &roadPtr,
        const std::shared_ptr<CFurnitureData> &frnPtr,
        const std::shared_ptr<CBridgeData> &bridgePtr);

    /*!
     * @brief ネットワーク追加(道路中心線複数本分)
     * @param[in] vecCenterLine 道路中心線のポインタと幅員群
     * @param[in] roadPtr       道路のポインタ
    */
    void add(
        const std::vector<std::shared_ptr<CCenterLineData>> &vecCenterLine,
        const std::shared_ptr<CTranRoadData> &roadPtr);

    /*!
     * @brief 関連道路情報管理用ハッシュマップの更新
     * @param[in] roadPtr 道路ポインタ
     * @param[in] centerLinePtr 中心線ポインタ
    */
    void updateRoadHashMap(
        const std::shared_ptr<CTranRoadData> &roadPtr,
        const std::shared_ptr<CCenterLineData> &centerLinePtr);

    /*!
     * @brief 関連都市設備情報管理用ハッシュマップの更新
     * @param[in] frnPtr 道路ポインタ
     * @param[in] centerLinePtr 中心線ポインタ
    */
    void updateFurnitureHashMap(
        const std::shared_ptr<CFurnitureData> &frnPtr,
        const std::shared_ptr<CCenterLineData> &centerLinePtr);

    /*!
     * @brief 関連橋梁情報管理用ハッシュマップの更新
     * @param[in] roadPtr 道路ポインタ
     * @param[in] centerLinePtr 中心線ポインタ
    */
    void updateBridgeHashMap(
        const std::shared_ptr<CBridgeData> &bridgePtr,
        const std::shared_ptr<CCenterLineData> &centerLinePtr);

    /*!
     * @brief 道路ネットワークデータのノード点判定処理
     * @param[in] v　頂点ディスクリプタ
     * @return 判定結果
     * @retval true    ノード点である(隣接道路との境界点 or 終端道路の端点)
     * @retval false   ノード点ではない
    */
    bool isNode(const BoostVertexDesc &v);

    /*!
     * @brief ID生成
     * @param[in] nJPZone   平面直角座標系
     * @param[in] pt        座標(平面直角座標系)
     * @param[in/out] map   連番管理マップ
     * @return ID文字列
    */
    std::string getId(
        const int nJPZone,
        const Boost3DPointHash &pt,
        std::map<std::string, std::map<Boost3DPointHash, int>> &map);

    /*!
     * @brief ネットワークデータの取得
     * @param[in]   nJPZone 平面直角座標系の系番号
     * @param[out]  nodes           ノード群
     * @param[out]  links           リンク群
     * @param[out]  nMaxLinkNum     ノード情報内の接続リンク数の最大本数
    */
    void getNetworkData(
        const int nJPZone,
        std::vector<Node> &nodes,
        std::vector<Link> &links,
        int &nMaxLinkNum);

    /*!
     * @brief ネットワークデータのリンク取得(マルチスレッド対応入り)
     * @param[in]     centerLinePtr 中心線
     * @param[in]     roadPtr       道路ポインタ
     * @param[in]     frnPtr        都市設備ポインタ
     * @param[in]     bridgePtr     橋梁ポインタ
     * @param[in]     nJPZone       平面直角座標系の系番号
     * @param[in/out] links         リンク情報
     * @param[in/out] linkIdMap     リンクIDの連番管理用
     * @param[in/out] nodeIdMap     ノードIDの連番管理用
     * @param[in/out] nodeDataMap   ノード管理用マップ
     * @param[out]    geoms         追加したリンクジオメトリ(エラーチェック用)
     * @return  リンク情報追加の有無
     * @retval  true    リンクを追加した
     * @retval  false   リンクを追加していない
    */
    bool getLink(
        const std::shared_ptr<CCenterLineData> &centerLinePtr,
        const std::shared_ptr<CTranRoadData> &roadPtr,
        const std::shared_ptr<CFurnitureData> &frnPtr,
        const std::shared_ptr<CBridgeData> &bridgePtr,
        const int nJPZone,
        std::vector<Link> &links,
        std::map<std::string, std::map<Boost3DPointHash, int>> &linkIdMap,
        std::map<std::string, std::map<Boost3DPointHash, int>> &nodeIdMap,
        std::unordered_map<BoostVertexDesc, Node> &nodeDataMap,
        Boost3DHashMultiLines &geoms);

    /*!
     * @brief ノード情報のフィールド情報作成
     * @param[in] nMaxLinkNum 接続リンクの最大本数
     * @param[in] dataType 車歩道判定用の種別
     * @return フィールド情報群
    */
    std::vector<CGISFileAttribute::AttributeFieldData> createNodeFields(
        const int nMaxLinkNum,
        const NETWORK_DATA_TYPE dataType);

    /*!
     * @brief リンク情報のフィールド情報作成
     * @param[in] dataType 車歩道判定用の種別
     * @return フィールド情報群
    */
    std::vector<CGISFileAttribute::AttributeFieldData> createLinkFields(const NETWORK_DATA_TYPE dataType);

    /*!
     * @brief ノード情報の幾何形状と属性情報の作成
     * @param[in]   nodes       ネットワークデータのノード情報
     * @param[in]   nMaxLinkNum 接続リンクの最大本数
     * @param[in]   dataType    車歩道判定用の種別
     * @param[in]   nJPZone     平面直角座標系の系番号
     * @param[out]  pts         ノードの幾何形状群
     * @param[out]  attrRecords ノードの属性情報群
    */
    void createNodeData(
        const std::vector<Node> &nodes,
        const int nMaxLinkNum,
        const NETWORK_DATA_TYPE dataType,
        const int nJPZone,
        Boost3DMultiPointHashs &pts,
        std::vector<CGISFileAttribute::AttributeDataRecord> &attrRecords);

    /*!
     * @brief リンク情報の幾何形状と属性情報の作成
     * @param[in]   links       ネットワークデータのリンク情報
     * @param[in]   dataType    車歩道判定用の種別
     * @param[in]   nJPZone     平面直角座標系の系番号
     * @param[in]   dataType 車歩道判定用の種別
     * @param[out]  polylines   リンクの幾何形状群
     * @param[out]  attrRecords リンクの属性情報群
    */
    void createLinkData(
        const std::vector<Link> &links,
        const NETWORK_DATA_TYPE dataType,
        const int nJPZone,
        Boost3DHashMultiLines &polylines,
        std::vector<CGISFileAttribute::AttributeDataRecord> &attrRecords);

    /*!
     * @brief 縦断勾配(コード値)の取得
     * @param[in]   nVtclSlope      縦断勾配値 %単位
     * @param[in]   isVEndHigher    起点より終点が高いか否か
     * @return      コード値
    */
    VTCL_SLOPE_TYPE getVtclSlopeType(const int nVtclSlope, const bool isVEndHigher);

    /*!
     * @brief 歩道用幅員(コード値)の取得
     * @param[in]   dWidth
     * @return      コード値
    */
    WIDTH_TYPE getWidthType(const double dWidth);

    /*!
     * @brief 経路構造の取得
     * @param[in] roadPtr   道路ポインタ
     * @param[in] frnPtr    都市設備ポインタ
     * @param[in] bridgePtr 橋梁ポインタ
     * @return 経路構造タイプ
    */
    ROUTE_STRUCTURE_TYPE getRouteStructureType(
        const std::shared_ptr<CTranRoadData> &roadPtr,
        const std::shared_ptr<CFurnitureData> &frnPtr,
        const std::shared_ptr<CBridgeData> &bridgePtr);

    /*!
     * @brief 幅員ランクの取得
     * @param[in] dWidth   幅員
     * @return ランク文字
    */
    std::string getWidthRank(const double dWidth);

    /*!
     * @brief 縦断勾配ランクの取得
     * @param[in] nVSlope   縦断勾配%
     * @return ランク文字
    */
    std::string getVSlopeRank(const int nVSlope);

    /*!
     * @brief 幅員の計測
     * @param[in]   line            中心線
     * @param[in]   roadPt          道路ポインタ
     * @param[in]   type            ネットワーク種別
     * @param[in]   dInterval       中心線のサンプリング間隔
     * @param[out]  minWidth        最小幅員
     * @param[out]  pt              最小幅員地点
     * @param[out]  isRefMinWidth   最小幅員が参考値の場合のフラグ
     * @return 測定結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool measureWidth(
        const Boost3DHashPolyline &line,
        const std::shared_ptr<CTranRoadData> &roadPtr,
        const NETWORK_DATA_TYPE type,
        const double dInterval,
        double &dMinWidth,
        Boost3DPointHash &pt,
        bool &isRefMinWidth);

    /*!
     * @brief 点字ブロックの有無の設定
     * @param[in]   line            中心線
     * @param[in]   roadPt          道路ポインタ
     * @param[in]   crossingPtr     横断歩道ポインタ
     * @param[in]   dDistTh         中心線のサンプリング点と近傍点字ブロック間の距離しきい値
     * @return  点字ブロックの有無
     * @retval  BRAILLE_TILE_TYPE::EXIST            有り
     * @retval  BRAILLE_TILE_TYPE::DOES_NOT_EXIST   無し
     * @retval  BRAILLE_TILE_TYPE::UNKNOWN          不明
     */
    BRAILLE_TILE_TYPE checkBrailleTile(
        const Boost3DHashPolyline &line,
        const std::shared_ptr<CTranRoadData> &roadPtr,
        const std::shared_ptr<CFurnitureData> &crossingPtr,
        const double dDistTh);

    /*!
     * @brief   勾配の計測(道路、横断歩道用)
     * @param[in]   line            中心線
     * @param[in]   roadPtr         中心線に関連する道路ポインタ
     * @param[in]   crossingPtr     中心線に関連する都市設備ポインタ(横断歩道)
     * @param[in]   bSamplingFlag   サンプリングフラグ
     * @param[in]   dInterval       サンプリング間隔
     * @param[out]  nMaxVSlope      最大勾配(整数)
     * @param[out]  dMaxVSlope      最大勾配(実数)
     * @param[out]  isEndHigher     始点よりも終点が高いか否か
     * @param[out]  vSlopePos       最大勾配地点
     * @param[out]  nAveVSlope      平均勾配
     * @return  処理結果
     * @retval  true    成功
     * @retval  false   失敗
     * @note roadPtr, crossingPtrはどちらかが有効値、他方はnullptrとする
    */
    bool calcGradient(
        const Boost3DHashPolyline &line,
        const std::shared_ptr<CTranRoadData> &roadPtr,
        const std::shared_ptr<CFurnitureData> &crossingPtr,
        const bool bSamplingFlag,
        const double dInterval,
        int &nMaxVSlope,
        double &dMaxVSlope,
        bool &isEndHigher,
        Boost3DPointHash &vSlopePos,
        int &nAveVSlope);

    /*!
     * @brief 勾配の計測(横断歩道橋用)
     * @param[in]   line            中心線
     * @param[in]   bridgePtr       中心線に関する橋梁ポインタ(横断歩道橋)
     * @param[in]   dInterval       サンプリング間隔
     * @param[out]  nMaxVSlope      最大勾配(整数)
     * @param[out]  dMaxVSlope      最大勾配(実数)
     * @param[out]  isEndHigher     始点よりも終点が高いか否か
     * @param[out]  vSlopePos       最大勾配地点
     * @param[out]  nAveVSlope      平均勾配
     * @return  処理結果
     * @retval  true    成功
     * @retval  false   失敗
    */
    bool calcGradient(
        const Boost3DHashPolyline &line,
        const std::shared_ptr<CBridgeData> &bridgePtr,
        const bool bSamplingFlag,
        const double dInterval,
        int &nMaxVSlope,
        double &dMaxVSlope,
        bool &isEndHigher,
        Boost3DPointHash &vSlopePos,
        int &nAveVSlope);

    /*!
     * @brief   縦断勾配の計測(道路、横断歩道、横断歩道橋共通)
     * @param[in]   line            中心線
     * @param[in]   roadPtr         中心線に関連する道路ポインタ
     * @param[in]   crossingPtr     中心線に関連する都市設備ポインタ(横断歩道)
     * @param[in]   bridgePtr       中心線に関する橋梁ポインタ(横断歩道橋)
     * @param[in]   dInterval       サンプリング間隔
     * @param[out]  nMaxVSlope      最大縦断勾配
     * @param[out]  isEndHigher     始点よりも終点が高いか否か
     * @param[out]  vSlopePos       最大縦断勾配地点
     * @param[out]  nAveVSlope      平均縦断勾配
     * @return  処理結果
     * @retval  true    成功
     * @retval  false   失敗
     * @note roadPtr, crossingPtr, bridgePtrはどれか1つが有効値、その他はnullptrとする
    */
    bool calcLongitudinalGradient(
        const Boost3DHashPolyline &line,
        const std::shared_ptr<CTranRoadData> &roadPtr,
        const std::shared_ptr<CFurnitureData> &crossingPtr,
        const std::shared_ptr<CBridgeData> &bridgePtr,
        const double dInterval,
        int &nMaxVSlope,
        bool &isEndHigher,
        Boost3DPointHash &vSlopePos,
        int &nAveVSlope);

    /*!
     * @brief 横断勾配の計測(道路、横断歩道、横断歩道橋共通)
     * @param[in]   line          中心線
     * @param[in]   roadPtr       中心線に関連する道路ポインタ
     * @param[in]   crossingPtr   中心線に関連する都市設備ポインタ(横断歩道)
     * @param[in]   bridgePtr     中心線に関連する橋梁ポインタ(横断歩道橋)
     * @param[in]   dInterval     サンプリング間隔
     * @param[out]  nMaxHSlope    最大横断勾配
     * @param[out]  hSlopePos     最大横断勾配位置
     * @return      処理結果
     * @retval      true    成功
     * @retval      false   失敗
     * @note roadPtr, crossingPtr, bridgePtrはどれか1つが有効値、その他はnullptrとする
    */
    bool calcCrossGradient(
        const Boost3DHashPolyline &line,
        const std::shared_ptr<CTranRoadData> &roadPtr,
        const std::shared_ptr<CFurnitureData> &crossingPtr,
        const std::shared_ptr<CBridgeData> &bridgePtr,
        const double dInterval,
        int &nMaxHSlope,
        Boost3DPointHash &hSlopePos);

    /*!
     * @brief リンク作成有無の確認
     * @param[in]   tranPtr         道路情報ポインタ
     * @param[in]   centerLines     リンクを作成した中心線群
     * @return リンク未作成ポリゴンの中点群
    */
    Boost3DMultiPointHashs checkLink(
        const std::shared_ptr<CTranRoadData> &tranPtr,
        const Boost3DHashMultiLines &centerLines);

};

#pragma endregion
