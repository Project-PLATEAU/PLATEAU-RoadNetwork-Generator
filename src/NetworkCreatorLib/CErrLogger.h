#pragma once
#include <string>
#include "Boost3DPointHash.h"

/*!
 * @brief 車道ネットワーク作成のエラー種別
*/
enum class RoadwayErrType
{
    OUTSIDE_OF_POLYGON = 0,             // 中心線がポリゴン外にはみ出ている
    GET_LINK_FAILED,                    // リンクの取得に失敗
};

/*!
 * @brief 歩道ネットワーク作成のエラー種別
*/
enum class FootpathErrType
{
    OUTSIDE_OF_POLYGON = 0,             // 中心線がポリゴン外にはみ出ている
    DISCONNECTED_PEDESTRIAN_CROSSING,   // 未接続横断歩道
    DISCONNECTED_PEDESTRIAN_BRIDGE,     // 未接続横断歩道橋
    GET_LINK_FAILED,                    // リンクの取得に失敗
};

/*!
 * @brief エラーログクラス
 * @note  ログファイルの書き出しはCLoggerの機能を使用
*/
class CErrLogger
{
private:
    static CErrLogger m_instance;       // 自クラス唯一のインスタンス
    std::string m_strRoadwayLogger;     // 車道用ロガー名
    std::string m_strFootpathLogger;    // 歩道用ロガー名

    /*!
     * @brief コンストラクタ
    */
    CErrLogger();

    /*!
     * @brief デストラクタ
    */
    virtual ~CErrLogger();

    /*!
     * @brief 車道用エラー種別に対応するエラーメッセージの取得
     * @param type  エラー種別
     * @return      エラーメッセージ
    */
    std::string getMessage(const RoadwayErrType type);

    /*!
     * @brief 歩道用エラー種別に対応するエラーメッセージの取得
     * @param type  エラー種別
     * @return      エラーメッセージ
    */
    std::string getMessage(const FootpathErrType type);



public:
    /*!
     * @brief インスタンスの取得
     * @return ロガーインスタンス
    */
    static CErrLogger *GetInstance() { return &m_instance; }

    /*!
     * @brief 初期化
    */
    void Initialize();

    /*!
     * @brief ログ書き出しを終了する
    */
    void Finish();

    /*!
     * @brief 車道用のエラーログ出力
     * @param type  エラー種別
     * @param pt    位置座標(平面直角座標系)
    */
    void WriteRoadwayLog(
        const RoadwayErrType type,
        const Boost3DPointHash &pt);

    /*!
     * @brief 車道用のエラーログ出力
     * @param type  エラー種別
     * @param dLat  緯度
     * @param dLon  経度
    */
    void WriteRoadwayLog(
        const RoadwayErrType type,
        const double dLat,
        const double dLon);

    /*!
     * @brief 歩道用のエラーログ出力
     * @param type  エラー種別
     * @param pt    位置座標(平面著各座標系)
    */
    void WriteFootpathLog(
        const FootpathErrType type,
        const Boost3DPointHash &pt);

    /*!
     * @brief 歩道用のエラーログ出力
     * @param type  エラー種別
     * @param dLat  緯度
     * @param dLon  経度
    */
    void WriteFootpathLog(
        const FootpathErrType type,
        const double dLat,
        const double dLon);

    /*!
     * @brief ポリゴンの代表点の取得
     * @param polygon ポリゴン
     * @return ポリゴンの中点または輪郭線内の1点
    */
    Boost3DPointHash GetRefPt(const Boost3DHashPolygon &polygon);

};

