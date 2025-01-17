#include "CErrLogger.h"
#include "CLogger.h"
#include "SettingData.h"
#include "CGeoUtil.h"
#include "CBoostGeoUtil.h"
#include "boost/format.hpp"

CErrLogger CErrLogger::m_instance;    // インスタンス

/*!
 * @brief コンストラクタ
*/
CErrLogger::CErrLogger()
    : m_strRoadwayLogger("roadwayErr"),
      m_strFootpathLogger("footpathErr")
{

}

/*!
 * @brief デストラクタ
*/
CErrLogger::~CErrLogger()
{

}

/*!
 * @brief 初期化
*/
void CErrLogger::Initialize()
{
    const std::string strPattern = "%v";
    const std::string strTitle = "lat,lon,err";  // 緯度, 経度,　エラーメッセージ

    // 車道用
    CLogger::GetInstance()->Initialize(
        COutputSettingData::GetInstance()->strRoadwayErrLogFilePath,
        m_strRoadwayLogger, spdlog::level::info, false, strPattern);
    CLogger::GetInstance()->WriteLog(strTitle, spdlog::level::info, m_strRoadwayLogger);

    if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD2
        || CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
    {
        // 歩道用
        CLogger::GetInstance()->Initialize(
            COutputSettingData::GetInstance()->strFootpathErrLogFilePath,
            m_strFootpathLogger, spdlog::level::info, false, strPattern);
        CLogger::GetInstance()->WriteLog(strTitle, spdlog::level::info, m_strFootpathLogger);
    }
}

/*!
 * @brief ログ書き出しを終了する
*/
void CErrLogger::Finish()
{
    // ロガーが存在する場合は削除
    // 車道用
    auto logger = spdlog::get(m_strRoadwayLogger);
    if (logger != nullptr)
    {
        spdlog::drop(m_strRoadwayLogger);
    }

    if (CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD2
        || CInputSettingData::GetInstance()->lodType == CInputSettingData::LODType::LOD3)
    {
        // 歩道用
        logger = spdlog::get(m_strFootpathLogger);
        if (logger != nullptr)
        {
            spdlog::drop(m_strFootpathLogger);
        }
    }
}

/*!
 * @brief 車道用のエラーログ出力
 * @param type  エラー種別
 * @param pt    位置座標(平面直角座標系)
*/
void CErrLogger::WriteRoadwayLog(
    const RoadwayErrType type,
    const Boost3DPointHash &pt)
{
    double dLat, dLon;
    CGeoUtil::XYToLatLon(
        CInputSettingData::GetInstance()->nJPZone,
        pt.y(), pt.x(), dLat, dLon);    // x:東西, y:南北
    WriteRoadwayLog(type, dLat, dLon);
}

/*!
 * @brief 車道用のエラーログ出力
 * @param type  エラー種別
 * @param dLat  緯度
 * @param dLon  経度
*/
void CErrLogger::WriteRoadwayLog(
    const RoadwayErrType type,
    const double dLat,
    const double dLon)
{
    std::string strErr = getMessage(type);
    std::string strMsg = (boost::format("%.15f,%.15f,%s") % dLat % dLon % strErr).str();
    CLogger::GetInstance()->WriteLog(strMsg, spdlog::level::info, m_strRoadwayLogger);
}

/*!
 * @brief 歩道用のエラーログ出力
 * @param type  エラー種別
 * @param pt    位置座標(平面著各座標系)
*/
void CErrLogger::WriteFootpathLog(
    const FootpathErrType type,
    const Boost3DPointHash &pt)
{
    double dLat, dLon;
    CGeoUtil::XYToLatLon(
        CInputSettingData::GetInstance()->nJPZone,
        pt.y(), pt.x(), dLat, dLon);    // x:東西, y:南北
    WriteFootpathLog(type, dLat, dLon);
}

/*!
 * @brief 歩道用のエラーログ出力
 * @param type  エラー種別
 * @param dLat  緯度
 * @param dLon  経度
*/
void CErrLogger::WriteFootpathLog(
    const FootpathErrType type,
    const double dLat,
    const double dLon)
{
    std::string strErr = getMessage(type);
    std::string strMsg = (boost::format("%.15f,%.15f,%s") % dLat % dLon % strErr).str();
    CLogger::GetInstance()->WriteLog(strMsg, spdlog::level::info, m_strFootpathLogger);
}

/*!
 * @brief 車道用エラー種別に対応するエラーメッセージの取得
 * @param type  エラー種別
 * @return      エラーメッセージ
*/
std::string CErrLogger::getMessage(const RoadwayErrType type)
{
    std::string strMsg = "";
    if (type == RoadwayErrType::OUTSIDE_OF_POLYGON)
    {
        strMsg = u8"リンク線がポリゴン外にはみ出している";
    }
    else if (type == RoadwayErrType::GET_LINK_FAILED)
    {
        strMsg = u8"リンク線の取得に失敗";
    }

    return strMsg;
}

/*!
 * @brief 歩道用エラー種別に対応するエラーメッセージの取得
 * @param type  エラー種別
 * @return      エラーメッセージ
*/
std::string CErrLogger::getMessage(const FootpathErrType type)
{
    std::string strMsg = "";

    if (type == FootpathErrType::OUTSIDE_OF_POLYGON)
    {
        strMsg = u8"リンク線がポリゴン外にはみ出している";
    }
    else if (type == FootpathErrType::DISCONNECTED_PEDESTRIAN_CROSSING)
    {
        strMsg = u8"未接続横断歩道のため要確認";
    }
    else if (type == FootpathErrType::DISCONNECTED_PEDESTRIAN_BRIDGE)
    {
        strMsg = u8"未接続横断歩道橋のため要確認";
    }
    else if (type == FootpathErrType::GET_LINK_FAILED)
    {
        strMsg = u8"リンク線の取得に失敗";
    }

    return strMsg;
}

/*!
 * @brief ポリゴンの代表点の取得
 * @param polygon ポリゴン
 * @return ポリゴンの中点または輪郭線内の1点
*/
Boost3DPointHash CErrLogger::GetRefPt(const Boost3DHashPolygon &polygon)
{
    Boost3DPointHash pt;
    bg::centroid(polygon, pt);
    if (CEpsUtil::Greater(bg::distance(polygon, pt), 0.001))
    {
        pt = polygon.outer().front();
    }
    return pt;
}
