#pragma once
#include <string>
#include <algorithm>
#include <iterator>
#include <vector>
#include "Boost3DPointHash.h"
#include "gdal/gdal.h"
#include "gdal/gdal_priv.h"
#include "gdal/ogrsf_frmts.h"

/*!
 * @brief EPSGユーティリティクラス
*/
class CEpsgUtil
{
public:
    /*!
     * @brief EPSGコード(使用する頻度が高いもののみ)
    */
    enum class EPSGCode
    {
        EPSG_JGD2011 = 6668,                    // JDG2011緯度経度
        EPSG_JGD2011_JPN_ZONE_1 = 6669,         // JDG2011平面直角座標1系
        EPSG_JGD2011_JPN_ZONE_2,                // JDG2011平面直角座標2系
        EPSG_JGD2011_JPN_ZONE_3,                // JDG2011平面直角座標3系
        EPSG_JGD2011_JPN_ZONE_4,                // JDG2011平面直角座標4系
        EPSG_JGD2011_JPN_ZONE_5,                // JDG2011平面直角座標5系
        EPSG_JGD2011_JPN_ZONE_6,                // JDG2011平面直角座標6系
        EPSG_JGD2011_JPN_ZONE_7,                // JDG2011平面直角座標7系
        EPSG_JGD2011_JPN_ZONE_8,                // JDG2011平面直角座標8系
        EPSG_JGD2011_JPN_ZONE_9,                // JDG2011平面直角座標9系
        EPSG_JGD2011_JPN_ZONE_10,               // JDG2011平面直角座標10系
        EPSG_JGD2011_JPN_ZONE_11,               // JDG2011平面直角座標11系
        EPSG_JGD2011_JPN_ZONE_12,               // JDG2011平面直角座標12系
        EPSG_JGD2011_JPN_ZONE_13,               // JDG2011平面直角座標13系
        EPSG_JGD2011_JPN_ZONE_14,               // JDG2011平面直角座標14系
        EPSG_JGD2011_JPN_ZONE_15,               // JDG2011平面直角座標15系
        EPSG_JGD2011_JPN_ZONE_16,               // JDG2011平面直角座標16系
        EPSG_JGD2011_JPN_ZONE_17,               // JDG2011平面直角座標17系
        EPSG_JGD2011_JPN_ZONE_18,               // JDG2011平面直角座標18系
        EPSG_JGD2011_JPN_ZONE_19,               // JDG2011平面直角座標系19
        EPSG_JGD2011_VERTICAL_HEIGHT = 6697,    // JDG2011緯度経度と東京湾平均海面を基準とする標高
    };

    /*!
     * @brief JDG2011平面直角座標系の系番号からEPSGコードを取得する
     * @param nJPZone   JDG2011平面直角座標系の系番号
     * @return EPSGコード(不正入力の場合は-1を返す)
    */
    static int GetEpsgFromJPZone(const int nJPZone)
    {
        if (0 < nJPZone && nJPZone < 20)
            return (6668 + nJPZone);
        else
            return -1;
    };

    /*!
     * @brie EPSGコード値の取得
     * @param code  EPSG enum class値
     * @return int型のEPSGコード
    */
    static int AsInt(const EPSGCode code)
    {
        return static_cast<std::underlying_type<EPSGCode>::type>(code);
    };
};




class CGISFileAttribute
{
public:

    /*!
     * @brief 属性値のフィールド定義型
    */
    enum class AttributeFieldType
    {
        ATTR_FIELD_TYPE_INT = 0,    //!< int型
        ATTR_FIELD_TYPE_DOUBLE,     //!< double型
        ATTR_FIELD_TYPE_STRING,     //!< 文字列型
    };

    /*!
     * @brief 属性値のデータ型
    */
    enum class AttributeDataType
    {
        ATTR_DATA_TYPE_NULL = 0,    //!< NULL型
        ATTR_DATA_TYPE_INT,         //!< int型
        ATTR_DATA_TYPE_DOUBLE,      //!< double型
        ATTR_DATA_TYPE_STRING,      //!< 文字列型
    };

    /*!
     * @brief 属性値のフィールド定義データ
    */
    struct AttributeFieldData
    {
        CGISFileAttribute::AttributeFieldType fieldType; //!< 属性値のフィールド定義型
        std::string strName;    //!< 属性名
        int nWidth;             //!< 全体の桁数
        int nDecimals;          //!< 小数部桁数


        /*!
         * コンストラクタ
         */
        AttributeFieldData()
        {
            fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_INT;
            strName = "";
            nWidth = 0;
            nDecimals = 0;
        }

        /*!
         * デストラクタ
         */
        virtual ~AttributeFieldData() {}

        /*!
         * コピーコンストラクタ
         */
        AttributeFieldData(const AttributeFieldData &x) { *this = x; }

        /*!
         * 代入演算子
         */
        AttributeFieldData &operator=(const AttributeFieldData &x)
        {
            if (this != &x)
            {
                fieldType = x.fieldType;
                strName = x.strName;
                nWidth = x.nWidth;
                nDecimals = x.nDecimals;
            }
            return *this;
        }

    };

    /*!
     * @brief 属性値データ
    */
    struct AttributeData
    {
        CGISFileAttribute::AttributeDataType dataType;   //!< 属性値のデータ型
        int nValue;                                 //!< 整数値
        double dValue;                              //!< 実数値
        std::string strValue;                       //!< 文字列

        /*!
         * コンストラクタ
         */
        AttributeData()
        {
            dataType = CGISFileAttribute::AttributeDataType::ATTR_DATA_TYPE_NULL;
            nValue = 0;
            dValue = 0;
            strValue = "";
        }

        /*!
         * @brief コンストラクタ
         * @param[in] nVal 整数値
         */
        AttributeData(int nVal)
        {
            dataType = CGISFileAttribute::AttributeDataType::ATTR_DATA_TYPE_INT;
            nValue = nVal;
            dValue = 0;
            strValue = "";
        }

        /*!
         * @brief コンストラクタ
         * @param[in] dVal 実数値
         */
        AttributeData(double dVal)
        {
            dataType = CGISFileAttribute::AttributeDataType::ATTR_DATA_TYPE_DOUBLE;
            nValue = 0;
            dValue = dVal;
            strValue = "";
        }

        /*!
         * @brief コンストラクタ
         * @param[in] str 文字列
         */
        AttributeData(std::string str)
        {
            dataType = CGISFileAttribute::AttributeDataType::ATTR_DATA_TYPE_STRING;
            nValue = 0;
            dValue = 0;
            strValue = str;
        }

        /*!
         * デストラクタ
         */
        virtual ~AttributeData() {}

        /*!
         * コピーコンストラクタ
         */
        AttributeData(const AttributeData &x) { *this = x; }

        /*!
         * 代入演算子
         */
        AttributeData &operator=(const AttributeData &x)
        {
            if (this != &x)
            {
                dataType = x.dataType;
                nValue = x.nValue;
                dValue = x.dValue;
                strValue = x.strValue;
            }
            return *this;
        }
    };

    /*!
     * @brief 1レコード分の属性値データ
    */
    struct AttributeDataRecord
    {
        int nShapeId;       //!< shape id
        std::vector<CGISFileAttribute::AttributeData> vecAttribute;  //!< 属性データ

        /*!
         * コンストラクタ
         */
        AttributeDataRecord()
        {
            nShapeId = 0;
        }

        /*!
         * デストラクタ
         */
        virtual ~AttributeDataRecord() {}

        /*!
         * コピーコンストラクタ
         */
        AttributeDataRecord(const AttributeDataRecord &x) { *this = x; }

        /*!
         * 代入演算子
         */
        AttributeDataRecord &operator=(const AttributeDataRecord &x)
        {
            if (this != &x)
            {
                nShapeId = x.nShapeId;
                std::copy(x.vecAttribute.begin(), x.vecAttribute.end(), std::back_inserter(vecAttribute));
            }
            return *this;
        }
    };
};

/*!
 * @brief ファイル書き込みクラス
*/
class CGISFileExporter
{
public:

    /*!
     * @brief 出力GISファイルタイプ
    */
    enum class GIS_FILE_TYPE
    {
        SHP = 0,
        GEOJSON,
    };

    CGISFileExporter(const GIS_FILE_TYPE type = GIS_FILE_TYPE::SHP);     //!< コンストラクタ
    ~CGISFileExporter(void) {}; //!< デストラクタ

    // ポリゴンのshape file出力
    bool OutputPolygons(
        const Boost3DHashMultiPolygon &polygons,
        std::string strShpPath,
        const std::vector<CGISFileAttribute::AttributeFieldData> &vecFields,
        const std::vector<CGISFileAttribute::AttributeDataRecord> &vecAttrRecords,
        const bool isUseZ = false,
        const int nEpsg = 6668,
        const std::string strEncoding="CP932");


    // ポリラインのshapefile出力
    bool OutputPolylines(
        const Boost3DHashMultiLines &polylines,
        std::string strShpPath,
        const std::vector<CGISFileAttribute::AttributeFieldData> &vecFields,
        const std::vector<CGISFileAttribute::AttributeDataRecord> &vecAttrRecords,
        const bool isUseZ = false,
        const int nEpsg = 6668,
        const std::string strEncoding = "CP932");

    // マルチポイントのshapfile出力
    bool OutputMultiPoints(
        const Boost3DMultiPointHashs &points,
        std::string strShpPath,
        const std::vector<CGISFileAttribute::AttributeFieldData> &vecFields,
        const std::vector<CGISFileAttribute::AttributeDataRecord> &vecAttrRecords,
        const bool isUseZ = false,
        const int nEpsg = 6668,
        const std::string strEncoding = "CP932");

    // マルチポリゴンのshape file出力
    bool OutputMultiPolygons(
        const std::vector< Boost3DHashMultiPolygon> &polygons,
        std::string strShpPath,
        const std::vector<CGISFileAttribute::AttributeFieldData> &vecFields,
        const std::vector<CGISFileAttribute::AttributeDataRecord> &vecAttrRecords,
        const bool isUseZ = false,
        const int nEpsg = 6668,
        const std::string strEncoding = "CP932");

protected:

private:
    GDALDriver *m_pDriver;              // ドライバ
    GDALDataset *m_pDataSet;            // データセット
    const std::string m_strEncoding;    // 文字コード指定時のkey

    /*!
     * @brief データセット作成
     * @param strshpPath 出力shpパス
     * @return 結果
     * @retval true  成功
     * @retval false 失敗
    */
    bool createDataSet(const std::string strShpPath);

    /*!
     * @brief データセットのクローズ
    */
    void closeDataSet();

    /*!
     * @brief 属性フィールドの設定
     * @param pLayer    レイヤ
     * @param vecFields 属性フィールド情報
    */
    void createField(OGRLayer *pLayer,
        const std::vector<CGISFileAttribute::AttributeFieldData> &vecFields);

    // 属性情報の書き込み
    bool writeAttribute(
        OGRFeature *pFeature,
        const std::vector<CGISFileAttribute::AttributeData> &vecAttributes);
};
