#pragma once

#include <citygml/citygml.h>

#include "BoostCommon.h"
#include "Boost3DPointHash.h"
#include "CCenterLineData.h"

/*!
 * @brief 道路を路線、同等以上の道路との交差点、道路構造の変化点で変化する場所で区切った区間における道路の構造
 * @note  i-URデータのため必ずデータが存在する訳ではない
*/
class CUroRoadStructureAttribute
{
public:
    std::string strWidthType;   // 幅員区分
    double dWidth;              // 幅員m
    int nNumberOfLanes;         // 上下線の合計車線数
    std::string strSectionType; // 道路構造の種別

    /*!
     * @brief コンストラクタ
     * @param strWidthType      幅員区分
     * @param dWidth            幅員m
     * @param nNumberOfLanes    上下線の合計車線数
     * @param strSectionType    道路構造の種別
    */
    CUroRoadStructureAttribute(
        std::string strWidthType = "",
        double dWidth = 0,
        int nNumberOfLanes = 0,
        std::string strSectionType = "")
    {
        this->strWidthType = strWidthType;
        this->dWidth = dWidth;
        this->nNumberOfLanes = nNumberOfLanes;
        this->strSectionType = strSectionType;
    }
};

class CTranRoadDataLodBase
{
public:
    CTranRoadDataLodBase() :
        m_iLod(0),
        m_boostGeometry(Boost3DHashPolygon())
    {

    }

    /// <summary>
    /// コピーコンストラクタ
    /// </summary>
    /// <param name="data"></param>
    CTranRoadDataLodBase(const CTranRoadDataLodBase& data) :
        m_iLod(data.m_iLod),
        m_boostGeometry(data.m_boostGeometry)
    {

    }

    virtual ~CTranRoadDataLodBase()
    {

    }

    CTranRoadDataLodBase& operator=(const CTranRoadDataLodBase& data)
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

class CTranRoadDataLod1 : public CTranRoadDataLodBase
{
public:
    CTranRoadDataLod1() :
        CTranRoadDataLodBase()
    {
        m_iLod = 1;
    }

    virtual ~CTranRoadDataLod1()
    {

    }

public:

};

class CTranRoadDataLod2 : public CTranRoadDataLodBase
{
public:
    CTranRoadDataLod2() :
        CTranRoadDataLodBase(),
        m_fuctionType(0)
        //m_trafficAreaFunctionType((TRAFFIC_AREA_FUNCTION_TYPE)0),
        //m_auxiliaryTrafficAreaFunctionType((AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE)0)
    {
        m_iLod = 2;
    }

    virtual ~CTranRoadDataLod2()
    {

    }

public:
    /**
     * 交通領域または交通補助領域のtran:function種別
     */
    int m_fuctionType;

    //TRAFFIC_AREA_FUNCTION_TYPE              m_trafficAreaFunctionType;          // 交通領域のtran:function種別
    //AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE    m_auxiliaryTrafficAreaFunctionType; // 交通補助領域のtran:function種別

};

class CTranRoadDataLod3 : public CTranRoadDataLodBase
{
public:
    CTranRoadDataLod3() :
        CTranRoadDataLodBase(),
        m_fuctionType(0),
        m_dLod3Type(3.0)
    {

    }

    virtual ~CTranRoadDataLod3()
    {

    }

public:
    /**
     * 交通領域または交通補助領域のtran:function種別
     */
    int m_fuctionType;

    /**
     * LOD3の詳細度
     */
    double m_dLod3Type;

private:

};

/*!
 * @brief 歩道データ
*/
class CFootpathData
{
public:
    /*!
     * @brief コンストラクタ
    */
    CFootpathData() {};

    /*!
     * @brief デストラクタ
    */
    ~CFootpathData() {};

    /**
     * コピーコンストラクタ
    */
    CFootpathData(const CFootpathData &x) { *this = x; }

    /**
     * 代入演算子
    */
    CFootpathData &operator = (const CFootpathData &data)
    {
        if (this != &data)
        {
            this->edgePairList = data.edgePairList;
            this->centerLineList = data.centerLineList;
        }
        return *this;
    }

    /*!
     * @brief 歩道縁
    */
    std::vector<std::pair<Boost3DHashPolyline, Boost3DHashPolyline>> edgePairList;

    /*!
     * @brief 中心線リスト
    */
    std::vector<std::shared_ptr<CCenterLineData>> centerLineList;

private:

};

/**
 * CityObjectを処理用に変換した
 * 交通CityGMLデータクラス
 */
class CTranRoadData
{
public:
    CTranRoadData() :
        m_roadStructureAttr(CUroRoadStructureAttribute()),
        m_nInOut(0),
        m_bIsCenterLineOnNeighborRoad(false)
    {
        m_strFunction = "";
        m_strId = "";
    }

    ~CTranRoadData()
    {

    }

    /**
     * コピーコンストラクタ
    */
    CTranRoadData(const CTranRoadData &x) { *this = x; }

    /**
     * 代入演算子
    */
    CTranRoadData &operator = (const CTranRoadData &road)
    {
        if (this != &road)
        {
            this->m_roadStructureAttr = road.m_roadStructureAttr;
            this->m_lod1 = road.m_lod1;
            this->m_lod2List = road.m_lod2List;
            this->m_lod3List = road.m_lod3List;
            this->m_lod3TriangularMeshList = road.m_lod3TriangularMeshList;
            this->m_neighborRoadPtr = road.m_neighborRoadPtr;
            this->m_nInOut = road.m_nInOut;
            this->edgePairList = road.edgePairList;
            this->roadCenterLineList = road.roadCenterLineList;
            this->m_bIsCenterLineOnNeighborRoad = road.m_bIsCenterLineOnNeighborRoad;
            this->m_strFunction = road.m_strFunction;
            this->m_strId = road.m_strId;
            this->m_footpath = road.m_footpath;
        }
        return *this;
    }

    /**
     * 道路構造
     */
    CUroRoadStructureAttribute              m_roadStructureAttr;

    /**
     * LOD1データ
     * 一つのCityObjectに一つのみ
     */
    CTranRoadDataLod1 m_lod1;

    /**
     * LOD2データ
     * 一つのCityObjectに複数
     */
    std::vector<CTranRoadDataLod2> m_lod2List;

    /**
     * LOD3データ
     * 一つのCityObjectに複数
     */
    std::vector<CTranRoadDataLod3> m_lod3List;

    /**
     * LOD3の三角メッシュデータ
     * 一つのCityObjectに複数
     */
    std::vector<CTranRoadDataLod3> m_lod3TriangularMeshList;

    /**
     * 隣接道路ポインタ群
     */
    std::set<std::shared_ptr<CTranRoadData>> m_neighborRoadPtr;

    /*!
     * @brief 道路の出入口数
     * @note  0:孤立, 1:行き止まり, 2:通路, 3以上:交差点
    */
    int m_nInOut;

    /**
     * 道路縁
    */
    std::vector<std::pair<Boost3DHashPolyline, Boost3DHashPolyline>> edgePairList;

    /**
     * 道路中心線リスト
     */
    std::vector<std::shared_ptr<CCenterLineData>> roadCenterLineList;

    /**
     * 隣接交差部の中心線設定の有無
     * （交差部の場合のみ）
     */
    bool m_bIsCenterLineOnNeighborRoad;

    /**
     * 道路の区分
    */
    std::string m_strFunction;

    /*!
     * @brief 路線名
    */
    std::string m_strName;

    /*!
     * @brief id
    */
    std::string m_strId;

    /*!
     * @brief 歩道データ
    */
    CFootpathData m_footpath;

private:

};