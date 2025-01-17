#include "CGISFileExporter.h"
#include "CFileUtil.h"
#include "CConvertGeometryUtil.h"
#include "gdal/ogr_spatialref.h"
#include "CUtil.h"

/*!
 * @brief コンストラクタ
*/
CGISFileExporter::CGISFileExporter(const CGISFileExporter::GIS_FILE_TYPE type)
    : m_strEncoding("ENCODING"),
    m_pDataSet(nullptr)
{
    GDALAllRegister();
    // ドライバ作成
    if (type == GIS_FILE_TYPE::GEOJSON)
    {
        m_pDriver = GetGDALDriverManager()->GetDriverByName("GeoJSON");
    }
    else
    {
        m_pDriver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
    }
}

/*!
 * @brief 初期化(ドライバとデータセット作成)
 * @param strshpPath 出力shpパス
 * @return 初期化結果
 * @retval true  成功
 * @retval false 失敗
*/
bool CGISFileExporter::createDataSet(const std::string strShpPath)
{
    std::string strUtf8 = CUtil::ConvShiftJisToUtf8(strShpPath);
    if (m_pDriver == nullptr)
        return false;   // 失敗

    m_pDataSet = m_pDriver->Create(strUtf8.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
    if (m_pDataSet == nullptr)
        return false;   // 失敗

    return true;    // 成功
}

/*!
 * @brief データセットのクローズ
*/
void CGISFileExporter::closeDataSet()
{
    if (m_pDataSet != nullptr)
    {
        GDALClose(m_pDataSet);
        m_pDataSet = nullptr;
    }
}

/*!
 * @brief 属性フィールドの設定
 * @param[in/out]   pLayer    レイヤ
 * @param[in]       vecFields 属性フィールド情報
*/
void CGISFileExporter::createField(OGRLayer *pLayer,
    const std::vector<CGISFileAttribute::AttributeFieldData> &vecFields)
{
    if (pLayer != nullptr)
    {
        for (const CGISFileAttribute::AttributeFieldData &data : vecFields)
        {
            OGRFieldType fieldType;
            switch (data.fieldType)
            {
                case CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_DOUBLE:   // 実数
                    fieldType = OGRFieldType::OFTReal;
                    break;
                case CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_STRING:   // 文字列
                    fieldType = OGRFieldType::OFTString;
                    break;
                case CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_INT:      // 整数
                default:
                    fieldType = OGRFieldType::OFTInteger;
                    break;
            }

            OGRFieldDefn fieldDef(data.strName.c_str(), fieldType);
            fieldDef.SetWidth(data.nWidth);
            if (fieldType == OGRFieldType::OFTReal)
                fieldDef.SetPrecision(data.nDecimals);  // 実数値の場合は小数部の桁数
            pLayer->CreateField(&fieldDef);     // 属性フィールドの作成
        }
    }
}

/*!
 * @brief ポリゴンのshape file出力
 * @param[in] polygons          ポリゴン群
 * @param[in] strShpPath        出力 shape file パス
 * @param[in] vecFields         属性フィールド定義データ
 * @param[in] vecAttrRecords    属性値データ
 * @param[in] isUseZ            z座標の出力可否
 * @param[in] nEpsg             座標系のEPSGコード
 * @param[in] strEncoding       出力時の文字コード(空文字の場合は文字コード未指定とする)
 * @return  処理結果
 * @retval  true        成功
 * @retval  false       失敗
*/
bool CGISFileExporter::OutputPolygons(
    const Boost3DHashMultiPolygon &polygons,
    std::string strShpPath,
    const std::vector<CGISFileAttribute::AttributeFieldData> &vecFields,
    const std::vector<CGISFileAttribute::AttributeDataRecord> &vecAttrRecords,
    const bool isUseZ,
    const int nEpsg,
    const std::string strEncoding)
{
    if (polygons.size() != vecAttrRecords.size())
        return false;

    // 座標系の設定
    OGRSpatialReference spatialRef;
    spatialRef.importFromEPSG(nEpsg);

    // 文字コード設定
    char **ppOptions = NULL;
    if (!strEncoding.empty())
        ppOptions = CSLSetNameValue(ppOptions, m_strEncoding.c_str(), strEncoding.c_str());

    // データセットの作成
    if (!createDataSet(strShpPath))
        return false;

    // レイヤ作成
    OGRLayer *pLayer;
    if (isUseZ)
        pLayer = m_pDataSet->CreateLayer("polygon", &spatialRef, wkbPolygonZM, ppOptions);
    else
        pLayer = m_pDataSet->CreateLayer("polygon", &spatialRef, wkbPolygon, ppOptions);

    if (pLayer == nullptr)
        return false;

    // 属性フィールドの作成
    createField(pLayer, vecFields);

    for (size_t id = 0; id < polygons.size(); id++)
    {
        // 幾何情報の設定
        OGRPolygon ogrPoly = CConvertGeometryUtil::ConvOgrPolygon(polygons[id], isUseZ);
        OGRFeature *pFeature = OGRFeature::CreateFeature(pLayer->GetLayerDefn());
        pFeature->SetGeometry(&ogrPoly);

        // 属性の設定
        writeAttribute(pFeature, vecAttrRecords[id].vecAttribute);

        // レイヤに追加
        pLayer->CreateFeature(pFeature);
        OGRFeature::DestroyFeature(pFeature);
    }

    closeDataSet();
    return true;
}


/*!
 * @brief ポリラインのshape file 出力
 * @param[in] polylines         ポリライン集合
 * @param[in] strShpPath        shpファイルパス
 * @param[in] vecFields         属性フィールド定義データ
 * @param[in] vecAttrRecords    属性値データ
 * @param[in] isUseZ            z座標の出力可否
 * @param[in] nEpsg             座標系のEPSGコード
 * @param[in] strEncoding       出力時の文字コード(空文字の場合は文字コード未指定とする)
 * @return  処理結果
 * @retval  true    成功
 * @retval  false   失敗
 */
bool CGISFileExporter::OutputPolylines(
    const Boost3DHashMultiLines &polylines,
    std::string strShpPath,
    const std::vector<CGISFileAttribute::AttributeFieldData> &vecFields,
    const std::vector<CGISFileAttribute::AttributeDataRecord> &vecAttrRecords,
    const bool isUseZ,
    const int nEpsg,
    const std::string strEncoding)
{
    if (polylines.size() != vecAttrRecords.size())
        return false;

    // 座標系の設定
    OGRSpatialReference spatialRef;
    spatialRef.importFromEPSG(nEpsg);

    // 文字コード設定
    char **ppOptions = NULL;
    if (!strEncoding.empty())
        ppOptions = CSLSetNameValue(ppOptions, m_strEncoding.c_str(), strEncoding.c_str());

    // データセットの作成
    if (!createDataSet(strShpPath))
        return false;

    // レイヤ作成
    OGRLayer *pLayer;
    if (isUseZ)
        pLayer = m_pDataSet->CreateLayer("polyline", &spatialRef, wkbLineStringZM, ppOptions);
    else
        pLayer = m_pDataSet->CreateLayer("polyline", &spatialRef, wkbLineString, ppOptions);
    if (pLayer == nullptr)
        return false;

    // 属性フィールドの作成
    createField(pLayer, vecFields);

    for (size_t id = 0; id < polylines.size(); id++)
    {
        // 幾何情報の設定
        OGRLineString ogrPolyline = CConvertGeometryUtil::ConvOgrLineString(polylines[id], isUseZ);
        OGRFeature *pFeature = OGRFeature::CreateFeature(pLayer->GetLayerDefn());
        pFeature->SetGeometry(&ogrPolyline);

        // 属性の設定
        writeAttribute(pFeature, vecAttrRecords[id].vecAttribute);

        // レイヤに追加
        pLayer->CreateFeature(pFeature);
        OGRFeature::DestroyFeature(pFeature);
    }

    closeDataSet();
    return true;
}

/*!
 * @brief 点のshape file出力
 * @param[in] points            点群
 * @param[in] strShpPath        shpファイルパス
 * @param[in] vecFields         属性フィールド定義データ
 * @param[in] vecAttrRecords    属性値データ
 * @param[in] isUseZ            z座標の出力可否
 * @param[in] nEpsg             座標系のEPSGコード
 * @param[in] strEncoding       出力時の文字コード(空文字の場合は文字コード未指定とする)
 * @return  処理結果
 * @retval  true    成功
 * @retval  false   失敗
*/
bool CGISFileExporter::OutputMultiPoints(
    const Boost3DMultiPointHashs &points,
    std::string strShpPath,
    const std::vector<CGISFileAttribute::AttributeFieldData> &vecFields,
    const std::vector<CGISFileAttribute::AttributeDataRecord> &vecAttrRecords,
    const bool isUseZ,
    const int nEpsg,
    const std::string strEncoding)
{
    if (points.size() != vecAttrRecords.size())
        return false;

    // 座標系の設定
    OGRSpatialReference spatialRef;
    spatialRef.importFromEPSG(nEpsg);

    // 文字コード設定
    char **ppOptions = NULL;
    if (!strEncoding.empty())
        ppOptions = CSLSetNameValue(ppOptions, m_strEncoding.c_str(), strEncoding.c_str());

    // データセットの作成
    if (!createDataSet(strShpPath))
        return false;

    // レイヤ作成
    OGRLayer *pLayer;
    if (isUseZ)
        pLayer = m_pDataSet->CreateLayer("point", &spatialRef, wkbPointZM, ppOptions);
    else
        pLayer = m_pDataSet->CreateLayer("point", &spatialRef, wkbPoint, ppOptions);
    if (pLayer == nullptr)
        return false;

    // 属性フィールドの作成
    createField(pLayer, vecFields);

    for (size_t id = 0; id < points.size(); id++)
    {
        // 幾何情報の設定
        OGRPoint ogrPoint = CConvertGeometryUtil::ConvOgrPoint(points[id], isUseZ);
        OGRFeature *pFeature = OGRFeature::CreateFeature(pLayer->GetLayerDefn());
        pFeature->SetGeometry(&ogrPoint);

        // 属性の設定
        writeAttribute(pFeature, vecAttrRecords[id].vecAttribute);

        // レイヤに追加
        pLayer->CreateFeature(pFeature);
        OGRFeature::DestroyFeature(pFeature);
    }

    closeDataSet();
    return true;
}

/*!
 * @brief マルチポリゴンのshape file出力
 * @param[in] polygons          マルチポリゴン群
 * @param[in] strShpPath        出力 shape file パス
 * @param[in] vecFields         属性フィールド定義データ
 * @param[in] vecAttrRecords    属性値データ
 * @param[in] isUseZ            z座標の出力可否
 * @param[in] nEpsg             座標系のEPSGコード
 * @param[in] strEncoding       出力時の文字コード(空文字の場合は文字コード未指定とする)
 * @return  処理結果
 * @retval  true        成功
 * @retval  false       失敗
*/
bool CGISFileExporter::OutputMultiPolygons(
    const std::vector<Boost3DHashMultiPolygon> &polygons,
    std::string strShpPath,
    const std::vector<CGISFileAttribute::AttributeFieldData> &vecFields,
    const std::vector<CGISFileAttribute::AttributeDataRecord> &vecAttrRecords,
    const bool isUseZ,
    const int nEpsg,
    const std::string strEncoding)
{
    if (polygons.size() != vecAttrRecords.size())
        return false;

    // 座標系の設定
    OGRSpatialReference spatialRef;
    spatialRef.importFromEPSG(nEpsg);

    // 文字コード設定
    char **ppOptions = NULL;
    if (!strEncoding.empty())
        ppOptions = CSLSetNameValue(ppOptions, m_strEncoding.c_str(), strEncoding.c_str());

    // データセットの作成
    if (!createDataSet(strShpPath))
        return false;

    // レイヤ作成
    OGRLayer *pLayer;
    if (isUseZ)
        pLayer = m_pDataSet->CreateLayer("polygon", &spatialRef, wkbPolygonZM, ppOptions);
    else
        pLayer = m_pDataSet->CreateLayer("polygon", &spatialRef, wkbPolygon, ppOptions);

    if (pLayer == nullptr)
        return false;

    // 属性フィールドの作成
    createField(pLayer, vecFields);

    for (size_t id = 0; id < polygons.size(); id++)
    {
        // 幾何情報の設定
        OGRMultiPolygon *pOgrMultiPoly = static_cast<OGRMultiPolygon *>(
            OGRGeometryFactory::createGeometry(OGRwkbGeometryType::wkbMultiPolygon));
        for (const auto &poly : polygons[id])
        {
            OGRPolygon *pOgrPoly = CConvertGeometryUtil::ConvOgrPolygonPtr(poly, isUseZ);
            pOgrMultiPoly->addGeometry(pOgrPoly);
        }

        OGRFeature *pFeature = OGRFeature::CreateFeature(pLayer->GetLayerDefn());
        pFeature->SetGeometry(pOgrMultiPoly);

        // 属性の設定
        writeAttribute(pFeature, vecAttrRecords[id].vecAttribute);

        // レイヤに追加
        pLayer->CreateFeature(pFeature);
        OGRFeature::DestroyFeature(pFeature);
        OGRGeometryFactory::destroyGeometry(pOgrMultiPoly);   // メモリ解放
    }

    closeDataSet();
    return true;
}

/*!
 * @brief 属性情報の書き込み
 * @param[in/out] pFeature      属性情報書き込み先データ
 * @param[in] vecAttributes     1幾何情報の属性データ
 * @return  処理結果
 * @retval  true    成功
 * @retval  false   失敗
*/
bool CGISFileExporter::writeAttribute(
    OGRFeature *pFeature,
    const std::vector<CGISFileAttribute::AttributeData> &vecAttributes)
{
    if (pFeature == nullptr)
        return false;

    for (size_t fid = 0; fid < vecAttributes.size(); fid++)
    {
        CGISFileAttribute::AttributeData data = vecAttributes[fid];
        OGRField field;
        switch (data.dataType)
        {
        case CGISFileAttribute::AttributeDataType::ATTR_DATA_TYPE_DOUBLE:   // 実数
            field.Real = data.dValue;
            break;
        case CGISFileAttribute::AttributeDataType::ATTR_DATA_TYPE_STRING:   // 文字列
            field.String = (char *)data.strValue.c_str();
            break;
        case CGISFileAttribute::AttributeDataType::ATTR_DATA_TYPE_INT:      // 整数
            field.Integer = data.nValue;
            break;
        default:
            break;
        }

        if (data.dataType == CGISFileAttribute::AttributeDataType::ATTR_DATA_TYPE_NULL)
        {
            pFeature->SetFieldNull(fid);    // NULL
        }
        else
        {
            pFeature->SetField(fid, &field);
        }
    }

    return true;
}
