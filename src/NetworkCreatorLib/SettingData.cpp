#include "SettingData.h"
#include "CTime.h"
#include "CFileUtil.h"

#pragma region InputSetting
CInputSettingData CInputSettingData::m_instance;    // インスタンス

/*!
 * @brief コンストラクタ
*/
CInputSettingData::CInputSettingData()
    : lodType(LODType::UNKNOWN),
      nJPZone(9)
{
}

/*!
 * @brief デストラクタ
*/
CInputSettingData::~CInputSettingData()
{
}

/*!
 * @brief 初期化(入力フォルダパスの設定)
 * @param strPath   入力フォルダのルートパス(CityGMLのトップ階層パス)
 * @param nLod      使用するLOD
 * @param nJPZone   平面直角座標系
*/
void CInputSettingData::Initialize(std::string strPath, int nLod, int nJPZone)
{
    // 想定
    // CityGMLトップ階層のフォルダ
    //  └ udx
    //      ├ brid  橋梁
    //      ├ frn   都市設備
    //      └ tran  交通(道路)
    std::string strUdxFolderPath = CFileUtil::Combine(strPath, "udx");
    strBridFolderPath = CFileUtil::Combine(strUdxFolderPath, "brid");
    strFrnFolderPath = CFileUtil::Combine(strUdxFolderPath, "frn");
    strTranFolderPath = CFileUtil::Combine(strUdxFolderPath, "tran");
    this->nJPZone = nJPZone;

    switch (nLod)
    {
        case 1:
            lodType = LODType::LOD1;
            break;
        case 2:
            lodType = LODType::LOD2;
            break;
        case 3:
            lodType = LODType::LOD3;
            break;
        default:
            lodType = LODType::UNKNOWN;
            break;
    }
}

#pragma endregion

#pragma region OutputSetting
COutputSettingData COutputSettingData::m_instance;    // インスタンス

/*!
 * @brief コンストラクタ
*/
COutputSettingData::COutputSettingData()
    : bCreateGeoJSON(false),
      bCreateSHP(false)
{
}

/*!
 * @brief デストラクタ
*/
COutputSettingData::~COutputSettingData()
{
}

/*!
 * @brief 初期化(出力フォルダパスの設定とフォルダ作成)
 * @param strPath 出力フォルダのルートパス
 * @param bSHP      SHP作成フラグ
 * @param bGeoJSON  GeoJSON作成フラグ
 * @param nLod      使用する道路のLOD
*/
void COutputSettingData::Initialize(std::string strPath, bool bSHP, bool bGeoJSON, int nLod)
{
    strOutputFolderPath = strPath;
    bCreateSHP = bSHP;
    bCreateGeoJSON = bGeoJSON;

    // 現在時刻
    std::string strTime = CTime::GetCurrentTime().Format("%Y%m%d_%H%M%S");
    strResultFolderPath = CFileUtil::Combine(strPath, strTime);

    // 出力フォルダパス
    std::string strSHPFolderPath = CFileUtil::Combine(strResultFolderPath, "SHP");
    std::string strGeoJSONFolderPath = CFileUtil::Combine(strResultFolderPath, "GeoJSON");
    strShpRoadwayFolderPath = CFileUtil::Combine(strSHPFolderPath, "roadway");
    strShpFootpathFolderPath = CFileUtil::Combine(strSHPFolderPath, "footpath");
    strGeoJsonRoadwayFolderPath = CFileUtil::Combine(strGeoJSONFolderPath, "roadway");
    strGeoJsonFootpathFolderPath = CFileUtil::Combine(strGeoJSONFolderPath, "footpath");

    // 出力フォルダの作成
    if (bSHP)
    {
        CFileUtil::CreateFolder(strShpRoadwayFolderPath);
        if (nLod > 1)
            CFileUtil::CreateFolder(strShpFootpathFolderPath);
    }
    if (bGeoJSON)
    {
        CFileUtil::CreateFolder(strGeoJsonRoadwayFolderPath);
        if (nLod > 1)
            CFileUtil::CreateFolder(strGeoJsonFootpathFolderPath);
    }

    // ログファイル
    strLogFilePath = CFileUtil::Combine(strResultFolderPath, "log.txt");
    strRoadwayErrLogFilePath = CFileUtil::Combine(strResultFolderPath, "errlog_roadway.csv");
    strFootpathErrLogFilePath = CFileUtil::Combine(strResultFolderPath, "errlog_footpath.csv");
}
#pragma endregion
