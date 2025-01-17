#include "CStatusManager.h"
#include "CEpsUtil.h"
#include "CLogger.h"
#include "boost/format.hpp"

CStatusManager CStatusManager::m_instance;    // インスタンス

/*!
    * @brief コンストラクタ
*/
CStatusManager::CStatusManager()
    : m_nProcessCnt(0),
      m_nNowStatus(-1),
      m_lodType(CInputSettingData::LODType::LOD1)
{

}

/*!
 * @brief デストラクタ
*/
CStatusManager::~CStatusManager()
{

}

/*!
 * @brief 初期化
 * @param[in] pCallback ステータスバー更新用コールバック関数
*/
void CStatusManager::Initialize(void *pCallback)
{
    m_pCallback = reinterpret_cast<callback_type>(pCallback);
}

/*!
 * @brief 進捗表示コールバック関数の呼び出し
 * @param dRate     進捗率(0 - 1.0)
 * @param isInit    進捗率を入力値で初期化するか否か
*/
void CStatusManager::UpdateStatus(double dRate, bool isInit)
{
    if (m_pCallback != nullptr)
    {
        m_pCallback(dRate, isInit);
    }
}

/*!
 * @brief ステータスを初期状態にする
 * @param lodType 入力LOD種別
*/
void CStatusManager::ResetStatus(CInputSettingData::LODType lodType)
{
    UpdateStatus(0, true);  // 進捗率0%に設定
    m_lodType = lodType;
    m_nNowStatus = -1;

    if (CInputSettingData::LODType::LOD1 == lodType)
    {
        // LOD1の場合
        m_nProcessCnt = static_cast<std::underlying_type<LOD1Process>::type>(LOD1Process::NUM);
    }
    else if (CInputSettingData::LODType::LOD2 == lodType)
    {
        // LOD2の場合
        m_nProcessCnt = static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::NUM);
    }
    else
    {
        // LOD3の場合
        m_nProcessCnt = static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::NUM);
    }

    std::queue<int> tmp;
    for (int i = 0; i < m_nProcessCnt; i++)
        tmp.push(i);
    m_statusQueue.swap(tmp);
}

/*!
 * @brief 1工程単位の進捗率の取得(SetProcessCntを呼び出してから使用すること)
 * @return 1工程単位の進捗率
*/
double CStatusManager::GetRatePerProcess()
{
    return 1.0 / static_cast<double>(m_nProcessCnt);
}

/*!
 * @brief 次工程のステータスに遷移する
 * @note ResetStatus後は現在ステータス未設定状態のため、
         最初の工程の前にNextStatusを呼び出しステータスを遷移すること
*/
void CStatusManager::NextStatus()
{
    // 現在ステータスを完了状態に遷移
    double dStep = static_cast<double>(m_nNowStatus);
    dStep += 1.0;   // ステータスenumが0オーダーのため1追加
    double dRate = GetRatePerProcess() * dStep;
    UpdateStatus(dRate, true);

    if (!m_statusQueue.empty())
    {
        // 現在ステータスを更新(先頭要素を取得)
        m_nNowStatus = m_statusQueue.front();
        m_statusQueue.pop();    // 先頭要素を削除

        std::string strNowStatus = NowStatus();
        CLogger::GetInstance()->WriteLog((boost::format("[%s]") % strNowStatus).str());
    }
}

/*!
 * @brief   現在のステータス取得
 * @return  ステータス名
*/
std::string CStatusManager::NowStatus()
{
    std::string strStatus = u8"未設定";

    if (m_lodType == CInputSettingData::LODType::LOD1)
    {
        strStatus = nowLod1Status();
    }
    else if (m_lodType == CInputSettingData::LODType::LOD2)
    {
        strStatus = nowLod2Status();
    }
    else if (m_lodType == CInputSettingData::LODType::LOD3)
    {
        strStatus = nowLod3Status();
    }

    return strStatus;
}

/*!
 * @brief   現在のステータス取得(LOD1用)
 * @return  ステータス名
*/
std::string CStatusManager::nowLod1Status()
{
    std::string strStatus = u8"未設定";

    switch (m_nNowStatus)
    {
    case static_cast<std::underlying_type<LOD1Process>::type>(LOD1Process::READ_ROAD_CITYGML):
        strStatus = u8"道路CityGMLの読み込み";
        break;
    case static_cast<std::underlying_type<LOD1Process>::type>(LOD1Process::SEARCH_NEIGHBOR_ROAD):
        strStatus = u8"隣接道路の探索";
        break;
    case static_cast<std::underlying_type<LOD1Process>::type>(LOD1Process::EXTRACT_ROAD_EDGE):
        strStatus = u8"車道のエッジ線抽出";
        break;
    case static_cast<std::underlying_type<LOD1Process>::type>(LOD1Process::CREATE_ROAD_CENTER_LINE):
        strStatus = u8"車道中心線の作成";
        break;
    case static_cast<std::underlying_type<LOD1Process>::type>(LOD1Process::CALC_ROAD_WIDTH):
        strStatus = u8"車道幅員の計測";
        break;
    case static_cast<std::underlying_type<LOD1Process>::type>(LOD1Process::CONNECT_ROAD_CENTER_LINE):
        strStatus = u8"車道の交差点接続";
        break;
    case static_cast<std::underlying_type<LOD1Process>::type>(LOD1Process::CONNECT_NEIGHBOR_ROAD):
        strStatus = u8"隣接道路接続";
        break;
    case static_cast<std::underlying_type<LOD1Process>::type>(LOD1Process::OUTPUT_ROAD_NW):
        strStatus = u8"車道ネットワーク出力";
        break;
    default:
        break;
    }

    return strStatus;
}

/*!
 * @brief   現在のステータス取得(LOD2用)
 * @return  ステータス名
*/
std::string CStatusManager::nowLod2Status()
{
    std::string strStatus = u8"未設定";

    switch (m_nNowStatus)
    {
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::READ_ROAD_CITYGML):
        strStatus = u8"道路CityGMLの読み込み";
        break;
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::READ_BRIDGE_CITYGML):
        strStatus = u8"橋梁CityGMLの読み込み";
        break;
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::READ_FURNITURE_CITYGML):
        strStatus = u8"都市設備CityGMLの読み込み";
        break;
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::SEARCH_NEIGHBOR_ROAD):
        strStatus = u8"隣接道路の探索";
        break;
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::EXTRACT_ROAD_EDGE):
        strStatus = u8"車道のエッジ線抽出";
        break;
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::CREATE_ROAD_CENTER_LINE):
        strStatus = u8"車道中心線の作成";
        break;
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::CALC_ROAD_WIDTH):
        strStatus = u8"車道幅員の計測";
        break;
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::CONNECT_ROAD_CENTER_LINE):
        strStatus = u8"車道の交差点接続";
        break;
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::CONNECT_NEIGHBOR_ROAD):
        strStatus = u8"隣接道路接続";
        break;
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::OUTPUT_ROAD_NW):
        strStatus = u8"車道ネットワーク出力";
        break;
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::EXTRACT_FOOTPATH_EDGE):
        strStatus = u8"歩道のエッジ線抽出";
        break;
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::CREATE_FOOTPATH_CENTER_LINE):
        strStatus = u8"歩道中心線の作成";
        break;
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::CONNECT_FOOTPATH_CENTER_LINE):
        strStatus = u8"歩道の交差点接続";
        break;
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::CREATE_PEDESTRIAN_CROSSING_CENTER_LINE):
        strStatus = u8"横断歩道の中心線作成";
        break;
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::CONNECT_PEDESTRIAN_CROSSING_CENTER_LINE):
        strStatus = u8"横断歩道による中心線接続";
        break;
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::CREATE_PEDESTRIAN_BRIDGE_CENTER_LINE):
        strStatus = u8"横断歩道橋の中心線作成";
        break;
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::CONNECT_PEDESTRIAN_BRIDGE_CENTER_LINE):
        strStatus = u8"横断歩道橋による中心線接続";
        break;
    case static_cast<std::underlying_type<LOD2Process>::type>(LOD2Process::OUTPUT_FOOTPATH_NW):
        strStatus = u8"歩道ネットワーク出力";
        break;
    default:
        break;
    }

    return strStatus;
}

/*!
 * @brief   現在のステータス取得(LOD3用)
 * @return  ステータス名
*/
std::string CStatusManager::nowLod3Status()
{
    std::string strStatus = u8"未設定";

    switch (m_nNowStatus)
    {
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::READ_ROAD_CITYGML):
        strStatus = u8"道路CityGMLの読み込み";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::READ_BRIDGE_CITYGML):
        strStatus = u8"橋梁CityGMLの読み込み";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::READ_FURNITURE_CITYGML):
        strStatus = u8"都市設備CityGMLの読み込み";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::SEARCH_NEIGHBOR_ROAD):
        strStatus = u8"隣接道路の探索";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::EXTRACT_ROAD_EDGE):
        strStatus = u8"車道のエッジ線抽出";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::CREATE_ROAD_CENTER_LINE):
        strStatus = u8"車道中心線の作成";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::CALC_ROAD_WIDTH):
        strStatus = u8"車道幅員の計測";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::CONNECT_ROAD_CENTER_LINE):
        strStatus = u8"車道の交差点接続";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::CONNECT_NEIGHBOR_ROAD):
        strStatus = u8"隣接道路接続";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::SET_ROAD_HEIGHT):
        strStatus = u8"車道中心線の標高値設定";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::OUTPUT_ROAD_NW):
        strStatus = u8"車道ネットワーク出力";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::EXTRACT_FOOTPATH_EDGE):
        strStatus = u8"歩道のエッジ線抽出";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::CREATE_FOOTPATH_CENTER_LINE):
        strStatus = u8"歩道中心線の作成";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::CONNECT_FOOTPATH_CENTER_LINE):
        strStatus = u8"歩道の交差点接続";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::CREATE_PEDESTRIAN_CROSSING_CENTER_LINE):
        strStatus = u8"横断歩道の中心線作成";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::CONNECT_PEDESTRIAN_CROSSING_CENTER_LINE):
        strStatus = u8"横断歩道による中心線接続";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::CREATE_PEDESTRIAN_BRIDGE_CENTER_LINE):
        strStatus = u8"横断歩道橋の中心線作成";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::CONNECT_PEDESTRIAN_BRIDGE_CENTER_LINE):
        strStatus = u8"横断歩道橋による中心線接続";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::SET_FOOTPATH_HEIGHT):
        strStatus = u8"歩道中心線の標高値設定";
        break;
    case static_cast<std::underlying_type<LOD3Process>::type>(LOD3Process::OUTPUT_FOOTPATH_NW):
        strStatus = u8"歩道ネットワーク出力";
        break;
    default:
        break;
    }

    return strStatus;
}