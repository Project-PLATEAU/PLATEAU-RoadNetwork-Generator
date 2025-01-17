#include "CGDALUtil.h"
#include "CConvertGeometryUtil.h"
#include "SettingData.h"
#include "CFileUtil.h"
#include "boost/format.hpp"
#include "gdal/gdal_utils.h"

CGDALUtil CGDALUtil::m_instance;    // インスタンス

/*!
 * @brief コンストラクタ
*/
CGDALUtil::CGDALUtil()
{
    GDALAllRegister();  // ツール起動中に呼び出しは1回が望ましい
}

/*!
 * @brief デストラクタ
*/
CGDALUtil::~CGDALUtil()
{
    GDALDestroy();
}

/*!
 * @brief ポリゴン融合
 * @param srcPolygons 融合するポリゴン群
 * @param isUseZ      z座標の使用可否
 * @param isRound     丸め座標の使用可否
 * @param nDigit      丸め座標使用時の小数点以下の桁数
 * @return 融合語のポリゴン群
 * @note   接しているポリゴンごとに融合する
*/
Boost3DHashMultiPolygon CGDALUtil::Dissolve(
    const Boost3DHashMultiPolygon &srcPolygons,
    const bool isUseZ,
    const bool isRound,
    const int nDigit)
{
    const std::string strPolygonType = "polygon";
    const std::string strDialect = "sqlite";
    const std::string strSqlHeader = "select GUnion(geometry) as geometry from ";
    Boost3DHashMultiPolygon dstPolygons;

    GDALDriver *pDriver = GetGDALDriverManager()->GetDriverByName("Memory");
    if (pDriver != nullptr)
    {
        GDALDataset *pDataSet = pDriver->Create("tmp", 0, 0, 0, GDT_Unknown, nullptr);
        if (pDataSet != nullptr)
        {
            OGRLayer *pLayer = pDataSet->CreateLayer(strPolygonType.c_str(), nullptr, wkbPolygon, nullptr);

            if (pLayer != nullptr)
            {
                for (const auto &srcPoly : srcPolygons)
                {
                    OGRPolygon ogrPoly = CConvertGeometryUtil::ConvOgrPolygon(
                        srcPoly, isUseZ, isRound, nDigit);
                    OGRFeature *pFeature = OGRFeature::CreateFeature(pLayer->GetLayerDefn());
                    pFeature->SetGeometry(&ogrPoly);
                    pLayer->CreateFeature(pFeature);
                    OGRFeature::DestroyFeature(pFeature);   // 削除しないとメモリリークする
                }

                // 融合
                // 衝突していないポリゴンも融合して1ジオメトリになる
                std::string strName(pLayer->GetName());
                std::string strSql = strSqlHeader + strName;
                OGRLayer *pDissolveLayer = pDataSet->ExecuteSQL(strSql.c_str(), nullptr, strDialect.c_str());

                if (pDissolveLayer != nullptr)
                {
                    //for (auto &pFeature : pDissolveLayer)
                    OGRFeature *pFeature;
                    while ((pFeature = pDissolveLayer->GetNextFeature()) != nullptr)
                    {
                        OGRGeometry *pGeom = pFeature->GetGeometryRef();
                        if (pGeom != nullptr)
                        {
                            if (wkbFlatten(pGeom->getGeometryType()) == wkbPolygon)
                            {
                                OGRPolygon *pPoly = pGeom->toPolygon();
                                Boost3DHashPolygon dstPoly = CConvertGeometryUtil::ConvBoostPolygon(*pPoly);
                                if (!bg::is_empty(dstPoly))
                                    dstPolygons.push_back(dstPoly);
                            }
                            else if (wkbFlatten(pGeom->getGeometryType()) == wkbMultiPolygon)
                            {
                                // マルチポリゴンは分割する
                                OGRMultiPolygon *pMPoly = pGeom->toMultiPolygon();
                                for (auto &pPoly : pMPoly)
                                {
                                    Boost3DHashPolygon dstPoly = CConvertGeometryUtil::ConvBoostPolygon(*pPoly);
                                    if (!bg::is_empty(dstPoly))
                                        dstPolygons.push_back(dstPoly);
                                }
                            }
                        }
                        OGRFeature::DestroyFeature(pFeature);   // 削除しないとメモリリークする
                    }
                }

                pDataSet->ReleaseResultSet(pDissolveLayer); // 解放しないとメモリリークする
            }
            GDALClose(pDataSet);
        }
    }

    return dstPolygons;
}

// ラスタライズ
cv::Mat CGDALUtil::Rasterize(
    double &dBaseX,
    double &dBaseY,
    double &dResoX,
    double &dResoY,
    const Boost3DHashPolygon &polygon,
    const double dResolution)
{
    const bool isUseZ = false;
    const bool isRound = false;
    const int nDigit = ncl_common_def::POINT_SIGNIFICANT_DIGITS;

    const std::string strTmpFolderPath = (boost::format("%s\\%s") % COutputSettingData::GetInstance()->strResultFolderPath % "tmp").str();
    const std::string strInputTiffPath = (boost::format("%s\\%s") % strTmpFolderPath % "img.tiff").str();
    const std::string strPolygonType = "polygon";
    CFileUtil::CreateFolder(strTmpFolderPath);

    // ラスタ画像サイズ
    int nWidth, nHeight;
    GetRasterImgSize(nWidth, nHeight, polygon, dResolution);
    cv::Mat img = cv::Mat::zeros(nHeight, nWidth, CV_8UC1);;

    GDALDriver *pMemDriver = GetGDALDriverManager()->GetDriverByName("Memory");
    if (pMemDriver != nullptr)
    {
        GDALDataset *pDataSet = pMemDriver->Create("tmp", 0, 0, 0, GDT_Unknown, nullptr);
        if (pDataSet != nullptr)
        {
            OGRLayer *pLayer = pDataSet->CreateLayer(strPolygonType.c_str(), nullptr, wkbPolygon, nullptr);

            if (pLayer != nullptr)
            {
                OGRPolygon ogrPoly = CConvertGeometryUtil::ConvOgrPolygon(
                    polygon, isUseZ, isRound, nDigit);
                OGRFeature *pFeature = OGRFeature::CreateFeature(pLayer->GetLayerDefn());
                pFeature->SetGeometry(&ogrPoly);
                pLayer->CreateFeature(pFeature);
                OGRFeature::DestroyFeature(pFeature);   // 削除しないとメモリリークする

                // ラスタライズオプション
                char **argv = nullptr;
                argv = CSLAddString(argv, "-l");
                argv = CSLAddString(argv, pLayer->GetName());
                argv = CSLAddString(argv, "-init");
                argv = CSLAddString(argv, "0");
                argv = CSLAddString(argv, "-ts");
                argv = CSLAddString(argv, (boost::format("%d") % nWidth).str().c_str());
                argv = CSLAddString(argv, (boost::format("%d") % nHeight).str().c_str());
                argv = CSLAddString(argv, "-ot");
                argv = CSLAddString(argv, "Byte");
                GDALRasterizeOptions *pOptions = GDALRasterizeOptionsNew(argv, nullptr);

                // ラスタ変換
                int nUsageErr;
                GDALDataset *pTiffDataSet = static_cast<GDALDataset *>(GDALRasterize(
                    strInputTiffPath.c_str(), nullptr, pDataSet, pOptions, &nUsageErr));    // todo:メモリ上のデータセットに出力したい
                if (pTiffDataSet != nullptr)
                {
                    double geoTrans[6];
                    if (pTiffDataSet->GetGeoTransform(geoTrans) == CE_None)
                    {
                        dBaseX = geoTrans[0];
                        dResoX = geoTrans[1];
                        dBaseY = geoTrans[3];
                        dResoY = geoTrans[5];
                    }

                    // RasterBandは1始まり
                    GDALRasterBand *pBand = pTiffDataSet->GetRasterBand(1);
                    if (pBand != nullptr)
                    {
                        int nSizeX = pBand->GetXSize();
                        int nSizeY = pBand->GetYSize();
                        uint8_t *pBuff = (uint8_t *)CPLMalloc(sizeof(uint8_t) * nSizeX * nSizeY);
                        pBand->RasterIO(
                            GDALRWFlag::GF_Read, 0, 0, nSizeX, nSizeY, pBuff, nSizeX, nSizeY, GDT_Byte, 0, 0);
                        cv::Mat tmpImg = cv::Mat(nSizeY, nSizeX, CV_8UC1, pBuff);
                        cv::threshold(tmpImg, img, 0, 255, cv::THRESH_OTSU);
                        CPLFree(pBuff);
                    }
                }
                GDALClose(pTiffDataSet);

                // メモリ解放
                GDALRasterizeOptionsFree(pOptions);
                CSLDestroy(argv);
            }

            GDALClose(pDataSet);
        }
    }
    CFileUtil::RemoveMultiFolder(strTmpFolderPath);
    return img;
}

// ラスタ画像サイズの算出
void CGDALUtil::GetRasterImgSize(
    int &nWidth,
    int &nHeight,
    const Boost3DHashPolygon &polygon,
    const double dResolution)
{
    Boost3DHashBox box;
    bg::envelope(polygon, box);
    nWidth = static_cast<int>((box.max_corner().x() - box.min_corner().x()) / dResolution) + 1;
    nHeight = static_cast<int>((box.max_corner().y() - box.min_corner().y()) / dResolution) + 1;
}

// ConvexHull
Boost3DHashPolygon CGDALUtil::ConvexHull(
    const Boost3DHashMultiPolygon &srcPolygons,
    const bool isUseZ,
    const bool isRound,
    const int nDigit)
{
    Boost3DHashPolygon dstPolygon;
    if (srcPolygons.size() > 0)
    {
        OGRMultiPolygon *pOgrMultiPoly = static_cast<OGRMultiPolygon *>(OGRGeometryFactory::createGeometry(OGRwkbGeometryType::wkbMultiPolygon));
        for (const auto &srcPoly : srcPolygons)
        {
            OGRPolygon *pOgrPoly = CConvertGeometryUtil::ConvOgrPolygonPtr(
                srcPoly, isUseZ, isRound, nDigit);
            pOgrMultiPoly->addGeometry(pOgrPoly);
        }

        OGRGeometry *pConvexHull = pOgrMultiPoly->ConvexHull();
        if (pConvexHull != nullptr)
        {
            if (wkbFlatten(pConvexHull->getGeometryType()) == wkbPolygon)
            {
                OGRPolygon *pPoly = pConvexHull->toPolygon();
                dstPolygon = CConvertGeometryUtil::ConvBoostPolygon(*pPoly);
            }
            else if (wkbFlatten(pConvexHull->getGeometryType()) == wkbMultiPolygon)
            {
                // マルチポリゴンは分割する
                OGRMultiPolygon *pMPoly = pConvexHull->toMultiPolygon();
                //for (auto &pPoly : pMPoly)
                //{
                //    Boost3DHashPolygon dstPoly = CConvertGeometryUtil::ConvBoostPolygon(*pPoly);
                //    if (!bg::is_empty(dstPoly))
                //        dstPolygons.push_back(dstPoly);
                //}
            }
            OGRGeometryFactory::destroyGeometry(pConvexHull);   // メモリ解放
        }

        OGRGeometryFactory::destroyGeometry(pOgrMultiPoly);   // メモリ解放
    }
    return dstPolygon;
}
