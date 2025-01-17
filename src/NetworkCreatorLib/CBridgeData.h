#pragma once
#include "CityGMLCommon.h"
#include "Boost3DPointHash.h"
#include "CCenterLineData.h"

/*!
 * @brief 橋梁データの基底クラス
*/
class CBridgeDataLodBase
{
public:
    /*!
     * @brief コンストラクタ
    */
    CBridgeDataLodBase() :
        m_iLod(0),
        m_boostGeometry(Boost3DHashPolygon())
    {

    }

    /// <summary>
    /// コピーコンストラクタ
    /// </summary>
    CBridgeDataLodBase(const CBridgeDataLodBase &data) { *this = data; }

    /*!
     * @brief デストラクタ
    */
    virtual ~CBridgeDataLodBase()
    {

    }

    /*!
     * @brief 代入演算子
    */
    CBridgeDataLodBase &operator=(const CBridgeDataLodBase &data)
    {
        if (this != &data)
        {
            m_iLod = data.m_iLod;
            m_boostGeometry = data.m_boostGeometry;
        }
        return *this;
    }

public:
    int						m_iLod;		            // LOD
    Boost3DHashPolygon      m_boostGeometry;	    // boostのジオメトリ（幾何情報）

private:
};

/*!
 * @brief LOD1橋梁データクラス
*/
class CBridgeDataLod1 : public CBridgeDataLodBase
{
public:
    /*!
     * @brief コンストラクタ
    */
    CBridgeDataLod1() :
        CBridgeDataLodBase()
    {
        m_iLod = 1;
    }

    /*!
     * @brief デストラクタ
    */
    virtual ~CBridgeDataLod1()
    {

    }

};

/*!
 * @brief LOD2橋梁データクラス
*/
class CBridgeDataLod2 : public CBridgeDataLodBase
{
public:
    /*!
     * @brief コンストラクタ
    */
    CBridgeDataLod2() :
        CBridgeDataLodBase()
    {
        m_iLod = 2;
    }

    /*!
     * @brief デストラクタ
    */
    virtual ~CBridgeDataLod2()
    {

    }
};

/*!
 * @brief 横断歩道橋の中心線
*/
class CBridgeCenterLineData
{
public:
    /*!
     * @brief コンストラクタ
    */
    CBridgeCenterLineData() :
        m_bFrontLinkagePt(false),
        m_bBackLinkagePt(false) {};

    /*!
     * @brief コンストラクタ
    */
    CBridgeCenterLineData(
        const Boost3DHashPolyline &centerLine,
        const bool bFrontLinkagePt,
        const bool bBackLinkagePt)
    {
        this->m_centerLine.centerLine = centerLine;
        this->m_bFrontLinkagePt = bFrontLinkagePt;
        this->m_bBackLinkagePt = bBackLinkagePt;
    };

    /*!
     * @brief コンストラクタ
    */
    CBridgeCenterLineData(
        const CCenterLineData &centerLine,
        const bool bFrontLinkagePt,
        const bool bBackLinkagePt)
    {
        this->m_centerLine = centerLine;
        this->m_bFrontLinkagePt = bFrontLinkagePt;
        this->m_bBackLinkagePt = bBackLinkagePt;
    };

    /*!
     * @brief コピーコンストラクタ
    */
    CBridgeCenterLineData(const CBridgeCenterLineData &x) { *this = x; }

    /*!
     * 代入演算子
    */
    CBridgeCenterLineData &operator = (const CBridgeCenterLineData &x)
    {
        if (this != &x)
        {
            this->m_centerLine = x.m_centerLine;
            this->m_bFrontLinkagePt = x.m_bFrontLinkagePt;
            this->m_bBackLinkagePt = x.m_bBackLinkagePt;
        }
        return *this;
    }

    /*!
     * @brief デストラクタ
    */
    ~CBridgeCenterLineData() {};

    CCenterLineData m_centerLine;       // 横断歩道橋の中心線
    bool m_bFrontLinkagePt;             // 始点の連結状態(true : 連結, false : 端点)
    bool m_bBackLinkagePt;              // 終点の連結状態(true : 連結, false : 端点)

};

/*!
 * @brief 横断歩道橋情報
*/
class CPedestrianBridgeData
{
public:
    /*!
     * @brief コンストラクタ
    */
    CPedestrianBridgeData() :
        m_bUse(true) {};

    /*!
     * @brief コピーコンストラクタ
    */
    CPedestrianBridgeData(const CPedestrianBridgeData &x) { *this = x; }

    /*!
     * 代入演算子
    */
    CPedestrianBridgeData &operator = (const CPedestrianBridgeData &x)
    {
        if (this != &x)
        {
            this->m_centerLines = x.m_centerLines;
            this->m_bUse = x.m_bUse;
        }
        return *this;
    }

    /*!
     * @brief デストラクタ
    */
    ~CPedestrianBridgeData() {};

    std::vector<CBridgeCenterLineData> m_centerLines;   // 横断歩道橋の中心線
    bool m_bUse;            // 使用可否

};

/*!
 * @brief CityObjectを処理用に変換した橋梁CityGMLデータクラス
*/
class CBridgeData
{
public:
    /*!
     * @brief コンストラクタ
    */
    CBridgeData()
    {
        m_dLod2Type = 0;
        m_functionType = BRIDGE_FUNCTION_TYPE::UNKNOWN;
        m_strId = "";
    }

    /*!
     * @brief デストラクタ
    */
    ~CBridgeData()
    {

    }

    /**
     * コピーコンストラクタ
    */
    CBridgeData(const CBridgeData &x) { *this = x; }

    /**
     * 代入演算子
    */
    CBridgeData &operator = (const CBridgeData &bridge)
    {
        if (this != &bridge)
        {
            this->m_lod1List = bridge.m_lod1List;
            this->m_lod2List = bridge.m_lod2List;
            this->m_lod2TriangularMeshList = bridge.m_lod2TriangularMeshList;
            this->m_dLod2Type = bridge.m_dLod2Type;
            this->m_functionType = bridge.m_functionType;
            this->m_strId = bridge.m_strId;
        }
        return *this;
    }

#pragma region CityGML情報
    /**
     * LOD1データ
     * 箱型モデルのため1つCityObjectに複数面
     */
    std::vector <CBridgeDataLod1> m_lod1List;

    /**
     * LOD2データ(三角メッシュを融合済みデータ)
     */
    std::vector<CBridgeDataLod2> m_lod2List;

    /**
     * LOD2データの三角メッシュデータ(標高値取得のためOuterCeilingSurface or OuterFloorSurfaceのみ)
     */
    std::vector<CBridgeDataLod2> m_lod2TriangularMeshList;

    /*!
     * @brief LOD2の詳細度
    */
    double m_dLod2Type;

    /*!
     * @brief 橋梁の主たる機能による区分(brid:function)
    */
    BRIDGE_FUNCTION_TYPE m_functionType;

    /*!
     * @brief id
    */
    std::string m_strId;

#pragma endregion CityGML情報

#pragma region 横断歩道橋の解析情報
    CPedestrianBridgeData m_pedestrianBridgeData;
#pragma endregion 横断歩道橋の解析情報

private:

};
