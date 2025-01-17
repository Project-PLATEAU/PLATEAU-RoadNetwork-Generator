#pragma once
#include "CityGMLCommon.h"
#include "Boost3DPointHash.h"
#include "CGeoUtil.h"
#include "CCenterLineData.h"

/*!
 * @brief 都市設備データクラス
*/
class CFurnitureDataLodBase
{
public:
    /*!
     * @brief コンストラクタ
    */
    CFurnitureDataLodBase() :
        m_iLod(0),
        m_boostGeometry(Boost3DHashPolygon())
    {

    }

    /// <summary>
    /// コピーコンストラクタ
    /// </summary>
    CFurnitureDataLodBase(const CFurnitureDataLodBase &data) { *this = data; }

    /*!
     * @brief デストラクタ
    */
    virtual ~CFurnitureDataLodBase()
    {

    }

    /*!
     * @brief 代入演算子
    */
    CFurnitureDataLodBase &operator=(const CFurnitureDataLodBase &data)
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
 * @brief LOD1都市設備データクラス
*/
class CFurnitureDataLod1 : public CFurnitureDataLodBase
{
public:
    /*!
     * @brief コンストラクタ
    */
    CFurnitureDataLod1() :
        CFurnitureDataLodBase()
    {
        m_iLod = 1;
    }

    /*!
     * @brief デストラクタ
    */
    virtual ~CFurnitureDataLod1()
    {

    }
};

/*!
 * @brief LOD2都市設備データクラス
*/
class CFurnitureDataLod2 : public CFurnitureDataLodBase
{
public:
    /*!
     * @brief コンストラクタ
    */
    CFurnitureDataLod2() :
        CFurnitureDataLodBase()
    {
        m_iLod = 2;
    }

    /*!
     * @brief デストラクタ
    */
    virtual ~CFurnitureDataLod2()
    {

    }
};

/*!
 * @brief LOD3都市設備データクラス
*/
class CFurnitureDataLod3 : public CFurnitureDataLodBase
{
public:
    /*!
     * @brief コンストラクタ
    */
    CFurnitureDataLod3() :
        CFurnitureDataLodBase()
    {
        m_iLod = 3;
    }

    /*!
     * @brief デストラクタ
    */
    virtual ~CFurnitureDataLod3()
    {

    }
};

/*!
 * @brief 横断歩道情報
*/
class CPedestrianCrossingData
{
public:
    /*!
     * @brief コンストラクタ
    */
    CPedestrianCrossingData():
        m_bStripes(false),
        m_dAve(0), m_dVar(0), m_dStd(0),
        m_bUse(true), m_bFrontConnection(false), m_bBackConnection(false) {};

    /*!
     * @brief コピーコンストラクタ
    */
    CPedestrianCrossingData(const CPedestrianCrossingData &x) { *this = x; }

    /*!
     * 代入演算子
    */
    CPedestrianCrossingData &operator = (const CPedestrianCrossingData &x)
    {
        if (this != &x)
        {
            this->m_bStripes = x.m_bStripes;
            this->m_centerLineData = x.m_centerLineData;
            this->m_mbr = x.m_mbr;
            this->m_directionOfMovement = x.m_directionOfMovement;
            this->m_rotateCenter = x.m_rotateCenter;
            this->m_dAve = x.m_dAve;
            this->m_dVar = x.m_dVar;
            this->m_dStd = x.m_dStd;
            this->m_bUse = x.m_bUse;
            this->m_bFrontConnection = x.m_bFrontConnection;
            this->m_bBackConnection = x.m_bBackConnection;
            //this->m_mbrs = x.m_mbrs;
            //this->m_simple = x.m_simple;
            //this->m_rects = x.m_rects;
            //this->m_src = x.m_src;
        }
        return *this;
    }

    /*!
     * @brief デストラクタ
    */
    ~CPedestrianCrossingData() {};

    /*!
     * @brief 中心線の取得
     * @return 中心線
    */
    std::shared_ptr<CCenterLineData> GetCenterLine()
    {
        return std::make_shared<CCenterLineData>(m_centerLineData);
    }

    bool m_bStripes;                        // 縞形状か否か
    CCenterLineData m_centerLineData;       // 中心線情報
    Boost3DHashPolygon m_mbr;               // Minimum Bounding Rectangle(横断歩道用)
    CVector2D m_directionOfMovement;        // 横断歩道の方向
    CVector2D m_rotateCenter;               // MBR作成時の回転中心
    double m_dAve;                          // 長辺方向の角度の平均
    double m_dVar;                          // 長辺方向の角度の分散
    double m_dStd;                          // 長辺方向の角度の標準偏差
    bool m_bUse;                            // 使用可否
    bool m_bFrontConnection;                // 始点の接続状況
    bool m_bBackConnection;                 // 終点の接続状況
    //Boost3DHashMultiPolygon m_mbrs;         // ジオメトリ個々のMBR
    //BoostMultiPolygon m_simple;             // 横断歩道の簡略化マルチポリゴン
    //BoostMultiPolygon m_rects;              // 全体MBRにおける長辺付近の領域(縦線形状判定用)
    //Boost3DHashMultiPolygon m_src;          // 入力ポリゴン群
};


/*!
 * @brief 都市設備データクラス
*/
class CFurnitureData
{
public:
    /*!
     * @brief コンストラクタ
    */
    CFurnitureData()
        : m_dLod3Type(0),
          m_functionType(FURNITURE_FUNCTION_TYPE::UNKNOWN)
    {
        m_strId = "";
    }

    /*!
     * @brief デストラクタ
    */
    virtual ~CFurnitureData()
    {

    }

    /**
     * コピーコンストラクタ
    */
    CFurnitureData(const CFurnitureData &x) { *this = x; }

    /**
     * 代入演算子
    */
    CFurnitureData &operator = (const CFurnitureData &furniture)
    {
        if (this != &furniture)
        {
            this->m_lod1List = furniture.m_lod1List;
            this->m_lod2List = furniture.m_lod2List;
            this->m_lod3List = furniture.m_lod3List;
            this->m_dLod3Type = furniture.m_dLod3Type;
            this->m_functionType = furniture.m_functionType;
            this->m_strId = furniture.m_strId;

            // 横断歩道の解析情報
            this->m_pedestrianCrossingData = furniture.m_pedestrianCrossingData;
        }
        return *this;
    }

#pragma region CityGML情報
    /**
     * LOD1データ
     */
    std::vector <CFurnitureDataLod1> m_lod1List;

    /**
     * LOD2データ
     */
    std::vector<CFurnitureDataLod2> m_lod2List;

    /**
     * LOD3データ
     */
    std::vector<CFurnitureDataLod3> m_lod3List;

    /*!
     * @brief LOD3の詳細度
    */
    double m_dLod3Type;

    /*!
     * @brief 都市設備の種類(frn:function)
    */
    FURNITURE_FUNCTION_TYPE m_functionType;

    /*!
     * @brief id
    */
    std::string m_strId;

#pragma endregion CityGML情報

#pragma region 横断歩道の解析情報
    CPedestrianCrossingData m_pedestrianCrossingData;
#pragma endregion 横断歩道の解析情報
};
