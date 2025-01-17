#include "CNetworkCreatorLib.h"
#include "CLogger.h"
#include "CErrLogger.h"
#include "CCityGMLReader.h"
#include "CStopWatch.h"
#include "CSearchNeighbor.h"
#include "CConvertGeometryUtil.h"
#include "CDebugUtil.h"
#include "CNetworkCreator.h"
#include "CNetwork.h"
#include "CStatusManager.h"
#include "CUtil.h"

/*!
 * @brief コンストラクタ
 * @param pCallback 進捗表示用コールバック関数ポインタ
*/
CNetworkCreatorLib::CNetworkCreatorLib(void *pCallback)
{
    // 進捗表示用コールバックの登録
    CStatusManager::GetInstance()->Initialize(pCallback);
}

/*!
 * @brief デストラクタ
*/
CNetworkCreatorLib::~CNetworkCreatorLib()
{

}

/*!
 * @brief 設定情報ログ出力
*/
void CNetworkCreatorLib::writeSettingLog()
{
    const CInputSettingData *pInput = CInputSettingData::GetInstance();
    const COutputSettingData *pOutput = COutputSettingData::GetInstance();

    // 入力設定
    CLogger::GetInstance()->WriteLog(u8"[入力設定]");
    CLogger::GetInstance()->WriteLog(u8"交通(道路) : " + CUtil::ConvShiftJisToUtf8(pInput->strTranFolderPath));

    if (pInput->lodType == CInputSettingData::LODType::LOD2 || pInput->lodType == CInputSettingData::LODType::LOD3)
    {
        CLogger::GetInstance()->WriteLog(u8"橋梁 : " + CUtil::ConvShiftJisToUtf8(pInput->strBridFolderPath));
        CLogger::GetInstance()->WriteLog(u8"都市設備 : " + CUtil::ConvShiftJisToUtf8(pInput->strFrnFolderPath));
    }

    CLogger::GetInstance()->WriteLog(
        u8"使用する道路 : LOD" + std::to_string(static_cast<int>(pInput->lodType)));

    // 出力設定
    CLogger::GetInstance()->WriteLog(u8"[出力設定]");
    if (pOutput->bCreateSHP)
    {
        CLogger::GetInstance()->WriteLog(u8"Shapefile出力 : あり");
        CLogger::GetInstance()->WriteLog(u8"道路 : " + CUtil::ConvShiftJisToUtf8(pOutput->strShpRoadwayFolderPath));
        if (pInput->lodType == CInputSettingData::LODType::LOD2 || pInput->lodType == CInputSettingData::LODType::LOD3)
            CLogger::GetInstance()->WriteLog(u8"歩道 : " + CUtil::ConvShiftJisToUtf8(pOutput->strShpFootpathFolderPath));
    }
    else
    {
        CLogger::GetInstance()->WriteLog(u8"Shapefile出力 : なし");
    }
    if (pOutput->bCreateGeoJSON)
    {
        CLogger::GetInstance()->WriteLog(u8"GeoJSON出力 : あり");
        CLogger::GetInstance()->WriteLog(u8"道路 : " + CUtil::ConvShiftJisToUtf8(pOutput->strGeoJsonRoadwayFolderPath));
        if (pInput->lodType == CInputSettingData::LODType::LOD2 || pInput->lodType == CInputSettingData::LODType::LOD3)
            CLogger::GetInstance()->WriteLog(u8"歩道 : " + CUtil::ConvShiftJisToUtf8(pOutput->strGeoJsonFootpathFolderPath));
    }
    else
    {
        CLogger::GetInstance()->WriteLog(u8"GeoJSON出力 : なし");
    }
    CLogger::GetInstance()->WriteLog(u8"ログファイル : " + CUtil::ConvShiftJisToUtf8(pOutput->strLogFilePath));
    CLogger::GetInstance()->WriteBorderLine();
}

// ネットワークデータ作成
bool CNetworkCreatorLib::CreateNetwork(
    const std::string &strInputDir,
    const std::string &strOutputDir,
    int nLod,
    int nJPZone,
    bool bSHP,
    bool bGeoJSON)
{
    CStopWatch sw;  // 全体処理時間計測用
    sw.Start();

    // 入力設定
    CInputSettingData::GetInstance()->Initialize(strInputDir, nLod, nJPZone);

    // 出力設定
    COutputSettingData::GetInstance()->Initialize(strOutputDir, bSHP, bGeoJSON, nLod);

    // ロガーの作成
#ifdef _DEBUG
    CLogger::GetInstance()->Initialize(COutputSettingData::GetInstance()->strLogFilePath, "main", spdlog::level::debug, true);
#else
    CLogger::GetInstance()->Initialize(COutputSettingData::GetInstance()->strLogFilePath);
#endif
    // エラーロガー作成(csvファイル)
    CErrLogger::GetInstance()->Initialize();

    // 入出力設定ログ
    writeSettingLog();

    // 進捗表示設定
    // 工程数が増減したらCStatusManager.hを修正すること
    CStatusManager::GetInstance()->ResetStatus(CInputSettingData::GetInstance()->lodType);
    CStatusManager::GetInstance()->NextStatus();    // 未設定状態から最初の工程に遷移

    // ネットワーク作成クラス
    CNetworkCreator creator = CNetworkCreator(nLod);

    // 道路CityGMLの読み込み
    double dLod3Detail = 0; // LOD3の場合の詳細度
    CCityGMLReader tranReader;
    tranReader.Init(CInputSettingData::GetInstance()->strTranFolderPath);
    double dCityGmlFileNum = static_cast<double>(tranReader.GetTargetCityGMLFileNum());
    double dRate = 1.0 / dCityGmlFileNum * CStatusManager::GetInstance()->GetRatePerProcess();   //全工程における1ファイル分の進捗率増分
    while (tranReader.GetTargetCityGMLFileNum() > 0)
    {
        try
        {
            // CityGML読み込み
            tranReader.Read();
        }
        catch (std::exception e)
        {
            CLogger::GetInstance()->WriteLog(e.what(), spdlog::level::err);
        }
        // 処理用データクラスに変換
        double dLod3Tmp;
        auto cityObjects = tranReader.GetCityObjects();
        creator.SetTranRoadData(cityObjects, CInputSettingData::GetInstance()->nJPZone, dLod3Tmp);
        if (CEpsUtil::Greater(dLod3Tmp, dLod3Detail))
            dLod3Detail = dLod3Tmp; // LOD3の場合の詳細度

        // 読み込み済みのCityGMLデータを解放
        tranReader.ReleaseCityModels();
        CStatusManager::GetInstance()->UpdateStatus(dRate, false);      // 進捗率の増分を加算
    }
    CStatusManager::GetInstance()->NextStatus();

    if (nLod > 1)
    {
        // 歩道ネットワークに必要なデータはLOD2道路以上を指定された場合に取得する
        // (歩道が判別可能なのはLOD2道路以上のため)
        // 橋梁CityGMLの読み込み
        CCityGMLReader bridgeReader;
        bridgeReader.Init(CInputSettingData::GetInstance()->strBridFolderPath);
        dCityGmlFileNum = static_cast<double>(bridgeReader.GetTargetCityGMLFileNum());
        dRate = 1.0 / dCityGmlFileNum * CStatusManager::GetInstance()->GetRatePerProcess();   //全工程における1ファイル分の進捗率増分
        while (bridgeReader.GetTargetCityGMLFileNum() > 0)
        {
            // CityGML読み込み
            bridgeReader.Read();

            // 処理用データクラスに変換
            auto cityObjects = bridgeReader.GetCityObjects();
            creator.SetBridgeData(cityObjects, CInputSettingData::GetInstance()->nJPZone);

            // 読み込み済みのCityGMLデータを解放
            bridgeReader.ReleaseCityModels();

            CStatusManager::GetInstance()->UpdateStatus(dRate, false);      // 進捗率の増分を加算
        }
        CStatusManager::GetInstance()->NextStatus();

        // 都市設備CityGML読み込み
        CCityGMLReader furnitureReader;
        furnitureReader.Init(CInputSettingData::GetInstance()->strFrnFolderPath);
        dCityGmlFileNum = static_cast<double>(furnitureReader.GetTargetCityGMLFileNum());
        dRate = 1.0 / dCityGmlFileNum * CStatusManager::GetInstance()->GetRatePerProcess();   //全工程における1ファイル分の進捗率増分
        while (furnitureReader.GetTargetCityGMLFileNum() > 0)
        {
            // CityGML読み込み
            furnitureReader.Read();

            // 処理用データクラスに変換
            auto cityObjects = furnitureReader.GetCityObjects();
            creator.SetFurnitureData(cityObjects, CInputSettingData::GetInstance()->nJPZone);

            // 読み込み済みのCityGMLデータを解放
            furnitureReader.ReleaseCityModels();

            CStatusManager::GetInstance()->UpdateStatus(dRate, false);      // 進捗率の増分を加算
        }
        CStatusManager::GetInstance()->NextStatus();
    }

    // 隣接道路探索
    creator.SearchNeighborRoad();
    CStatusManager::GetInstance()->NextStatus();

    // 車道
    {
        // エッジ線検出
        creator.DetectEdge();
        CStatusManager::GetInstance()->NextStatus();

        // 道路中心線作成
        creator.CreateCenterLines();
        CStatusManager::GetInstance()->NextStatus();

        // 幅員測定
        creator.MeasureRoadWidth();
        CStatusManager::GetInstance()->NextStatus();

        // 交差部接続
        creator.ConnectIntersection();
        CStatusManager::GetInstance()->NextStatus();

        // 隣接道路接続
        creator.ConnectNeighborRoad();
        CStatusManager::GetInstance()->NextStatus();

        if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
        {
            // 標高設定
            creator.SetRoadwayHeight();
            CStatusManager::GetInstance()->NextStatus();

        }

        // 車道ネットワーク出力
        creator.OutputRoadwayNetwork(
            COutputSettingData::GetInstance()->strShpRoadwayFolderPath,
            COutputSettingData::GetInstance()->strGeoJsonRoadwayFolderPath,
            CInputSettingData::GetInstance()->nJPZone,
            COutputSettingData::GetInstance()->bCreateSHP,
            COutputSettingData::GetInstance()->bCreateGeoJSON,
            dLod3Detail);
        CStatusManager::GetInstance()->NextStatus();
    }

    // 歩道
    if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD2
        || CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
    {
        // エッジ線検出
        creator.DetectEdgeOfFootpath();
        CStatusManager::GetInstance()->NextStatus();

        // 中心線作成
        creator.CreateFootpathCenterLines();
        CStatusManager::GetInstance()->NextStatus();

        // 交差部接続
        creator.FootpathConnectionByCrossing();
        CStatusManager::GetInstance()->NextStatus();

        // 横断歩道の中心線作成
        creator.CreateCenterLineOfPedestrianCrossing();
        CStatusManager::GetInstance()->NextStatus();

        // 横断歩道による接続
        creator.FootpathConnectionByPedestrianCrossing();
        CStatusManager::GetInstance()->NextStatus();

        // 横断歩道橋の中心線作成
        creator.CreateCenterLineOfPedestrianBridge();
        CStatusManager::GetInstance()->NextStatus();

        // 横断歩道橋による接続
        creator.FootpathConnectionByPedestrianBridge();
        CStatusManager::GetInstance()->NextStatus();

        // 標高設定
        if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
        {
            creator.SetFootpathHeight();
            CStatusManager::GetInstance()->NextStatus();
        }

        // 歩道ネットワーク出力
        creator.OutputFootpathNetwork(
            COutputSettingData::GetInstance()->strShpFootpathFolderPath,
            COutputSettingData::GetInstance()->strGeoJsonFootpathFolderPath,
            CInputSettingData::GetInstance()->nJPZone,
            COutputSettingData::GetInstance()->bCreateSHP,
            COutputSettingData::GetInstance()->bCreateGeoJSON,
            dLod3Detail);
        CStatusManager::GetInstance()->NextStatus();
    }

    sw.Stop();
    CLogger::GetInstance()-> WriteBorderLine();
    CLogger::GetInstance()->WriteLog("process time " + sw.ToString());
    CLogger::GetInstance()->Finish();
    CErrLogger::GetInstance()->Finish();

    return true;
}
