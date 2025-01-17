#include "CDebugUtil.h"
#include "CGISFileExporter.h"

/*!
 * @brief ポリゴンのshape file出力
 * @param[in] polygons          ポリゴン群
 * @param[in] strShpPath        出力 shape file パス
 * @param[in] nEpsg             座標系のEPSGコード
 * @param[in] strEncoding       出力時の文字コード(空文字の場合は文字コード未指定とする)
 * @return  処理結果
 * @retval  true        成功
 * @retval  false       失敗
*/
bool CDebugUtil::OutputPolygonsToShp(
    const Boost3DHashMultiPolygon &polygons,
    std::string strShpPath,
    const bool isUseZ,
    const int nEpsgCode,
    const std::string strEncoding)
{
    std::vector<CGISFileAttribute::AttributeFieldData> vecFields;
    std::vector<CGISFileAttribute::AttributeDataRecord> vecAttrRecords;

    CGISFileAttribute::AttributeFieldData field1;
    field1.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_INT;
    field1.strName = "id";
    field1.nWidth = 8;
    vecFields.push_back(field1);

    int nId = 0;
    for (Boost3DHashPolygon polygon : polygons)
    {
        CGISFileAttribute::AttributeDataRecord record;
        record.nShapeId = nId;
        record.vecAttribute.push_back(CGISFileAttribute::AttributeData(nId));
        vecAttrRecords.push_back(record);
        nId++;
    }

    CGISFileExporter exporter(CGISFileExporter::GIS_FILE_TYPE::SHP);
    return exporter.OutputPolygons(
        polygons, strShpPath, vecFields, vecAttrRecords, isUseZ, nEpsgCode, strEncoding);
}

/*!
 * @brief ポイントのshape file出力
 * @param[in] points            頂点群
 * @param[in] strShpPath        出力 shape file パス
 * @param[in] isUseZ            z座標の出力可否
 * @param[in] nEpsg             座標系のEPSGコード
 * @param[in] strEncoding       出力時の文字コード(空文字の場合は文字コード未指定とする)
 * @return  処理結果
 * @retval  true        成功
 * @retval  false       失敗
*/
bool CDebugUtil::OutputPointsToShp(
    const Boost3DMultiPointHashs &points,
    std::string strShpPath,
    const bool isUseZ,
    const int nEpsgCode,
    const std::string strEncoding)
{
    std::vector<CGISFileAttribute::AttributeFieldData> vecFields;
    std::vector<CGISFileAttribute::AttributeDataRecord> vecAttrRecords;

    CGISFileAttribute::AttributeFieldData field1;
    field1.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_INT;
    field1.strName = "id";
    field1.nWidth = 8;
    vecFields.push_back(field1);

    int nId = 0;
    for (Boost3DPointHash pt : points)
    {
        CGISFileAttribute::AttributeDataRecord record;
        record.nShapeId = nId;
        record.vecAttribute.push_back(CGISFileAttribute::AttributeData(nId));
        vecAttrRecords.push_back(record);
        nId++;
    }

    CGISFileExporter exporter(CGISFileExporter::GIS_FILE_TYPE::SHP);
    return exporter.OutputMultiPoints(
        points, strShpPath, vecFields, vecAttrRecords, isUseZ, nEpsgCode, strEncoding);
}

/*!
 * @brief ポリラインのshape file出力
 * @param[in] polylines         ポリライン群
 * @param[in] widthList         幅員リスト
 * @param[in] strShpPath        出力 shape file パス
 * @param[in] nEpsg             座標系のEPSGコード
 * @param[in] strEncoding       出力時の文字コード(空文字の場合は文字コード未指定とする)
 * @return  処理結果
 * @retval  true        成功
 * @retval  false       失敗
*/
bool CDebugUtil::OutputPolylinesToShp(
    const Boost3DHashMultiLines &polylines,
    std::string strShpPath,
    std::vector<double> widthList,
    const bool isUseZ,
    const int nEpsgCode,
    const std::string strEncoding)
{
    bool isWidth = (widthList.size() != 0);

    std::vector<CGISFileAttribute::AttributeFieldData> vecFields;
    std::vector<CGISFileAttribute::AttributeDataRecord> vecAttrRecords;

    CGISFileAttribute::AttributeFieldData field1;
    field1.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_INT;
    field1.strName = "id";
    field1.nWidth = 8;
    vecFields.push_back(field1);

    if (isWidth)
    {
        CGISFileAttribute::AttributeFieldData field2;
        field2.fieldType = CGISFileAttribute::AttributeFieldType::ATTR_FIELD_TYPE_DOUBLE;
        field2.strName = "width";
        field2.nWidth = 8;
        vecFields.push_back(field2);
    }

    int nId = 0;
    for (Boost3DHashPolyline polyline : polylines)
    {
        CGISFileAttribute::AttributeDataRecord record;
        record.nShapeId = nId;
        record.vecAttribute.push_back(CGISFileAttribute::AttributeData(nId));

        if (isWidth && nId < widthList.size())
        {
            record.vecAttribute.push_back(CGISFileAttribute::AttributeData(widthList[nId]));
        }

        vecAttrRecords.push_back(record);
        nId++;
    }

    CGISFileExporter exporter(CGISFileExporter::GIS_FILE_TYPE::SHP);
    return exporter.OutputPolylines(
        polylines, strShpPath, vecFields, vecAttrRecords, isUseZ, nEpsgCode, strEncoding);
}