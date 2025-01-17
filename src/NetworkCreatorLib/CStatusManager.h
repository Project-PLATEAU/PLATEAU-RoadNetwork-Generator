#pragma once
#include <queue>
#include <string>
#include "SettingData.h"

/*!
 * @brief ステータス管理クラス(Singleton)
*/
class CStatusManager
{
public:

    /*!
     * @brief LOD1の作業工程
    */
    enum class LOD1Process
    {
        READ_ROAD_CITYGML = 0,      // 道路CityGMLの読み込み
        SEARCH_NEIGHBOR_ROAD,       // 隣接道路探索
        EXTRACT_ROAD_EDGE,          // 車道エッジ線抽出
        CREATE_ROAD_CENTER_LINE,    // 車道中心線作成
        CALC_ROAD_WIDTH,            // 車道幅員計測
        CONNECT_ROAD_CENTER_LINE,   // 車道交差点接続
        CONNECT_NEIGHBOR_ROAD,      // 隣接道路接続
        OUTPUT_ROAD_NW,             // 車道ネットワーク出力
        NUM,                        // 工程数
    };

    /*!
     * @brief LOD2の作業工程
    */
    enum class LOD2Process
    {
        READ_ROAD_CITYGML = 0,                      // 道路CityGMLの読み込み
        READ_BRIDGE_CITYGML,                        // 橋梁CityGMLの読み込み
        READ_FURNITURE_CITYGML,                     // 都市設備CityGMLの読み込み
        SEARCH_NEIGHBOR_ROAD,                       // 隣接道路探索
        EXTRACT_ROAD_EDGE,                          // 車道エッジ線抽出
        CREATE_ROAD_CENTER_LINE,                    // 車道中心線作成
        CALC_ROAD_WIDTH,                            // 車道幅員計測
        CONNECT_ROAD_CENTER_LINE,                   // 車道交差点接続
        CONNECT_NEIGHBOR_ROAD,                      // 隣接道路接続
        OUTPUT_ROAD_NW,                             // 車道ネットワーク出力
        EXTRACT_FOOTPATH_EDGE,                      // 歩道エッジ線抽出
        CREATE_FOOTPATH_CENTER_LINE,                // 歩道中心線作成
        CONNECT_FOOTPATH_CENTER_LINE,               // 歩道交差点接続
        CREATE_PEDESTRIAN_CROSSING_CENTER_LINE,     // 横断歩道の中心線作成
        CONNECT_PEDESTRIAN_CROSSING_CENTER_LINE,    // 横断歩道による接続
        CREATE_PEDESTRIAN_BRIDGE_CENTER_LINE,       // 横断歩道橋の中心線作成
        CONNECT_PEDESTRIAN_BRIDGE_CENTER_LINE,      // 横断歩道橋による接続
        OUTPUT_FOOTPATH_NW,                         // 歩道ネットワーク出力
        NUM,                                        // 工程数
    };


    /*!
     * @brief LOD3.0-3.3の作業工程
    */
    enum class LOD3Process
    {
        READ_ROAD_CITYGML = 0,                      // 道路CityGMLの読み込み
        READ_BRIDGE_CITYGML,                        // 橋梁CityGMLの読み込み
        READ_FURNITURE_CITYGML,                     // 都市設備CityGMLの読み込み
        SEARCH_NEIGHBOR_ROAD,                       // 隣接道路探索
        EXTRACT_ROAD_EDGE,                          // 車道エッジ線抽出
        CREATE_ROAD_CENTER_LINE,                    // 車道中心線作成
        CALC_ROAD_WIDTH,                            // 車道幅員計測
        CONNECT_ROAD_CENTER_LINE,                   // 車道交差点接続
        CONNECT_NEIGHBOR_ROAD,                      // 隣接道路接続
        SET_ROAD_HEIGHT,                            // 車道の標高値設定
        OUTPUT_ROAD_NW,                             // 車道ネットワーク出力
        EXTRACT_FOOTPATH_EDGE,                      // 歩道エッジ線抽出
        CREATE_FOOTPATH_CENTER_LINE,                // 歩道中心線作成
        CONNECT_FOOTPATH_CENTER_LINE,               // 歩道交差点接続
        CREATE_PEDESTRIAN_CROSSING_CENTER_LINE,     // 横断歩道の中心線作成
        CONNECT_PEDESTRIAN_CROSSING_CENTER_LINE,    // 横断歩道による接続
        CREATE_PEDESTRIAN_BRIDGE_CENTER_LINE,       // 横断歩道橋の中心線作成
        CONNECT_PEDESTRIAN_BRIDGE_CENTER_LINE,      // 横断歩道橋による接続
        SET_FOOTPATH_HEIGHT,                        // 歩道の標高値設定
        OUTPUT_FOOTPATH_NW,                         // 歩道ネットワーク出力
        NUM,                                        // 工程数
    };

    using callback_type = void(__stdcall *)(double dRate, bool isInit);  // コールバック関数型

    /*!
     * @brief インスタンスの取得
     * @return ステータス管理インスタンス
    */
    static CStatusManager *GetInstance() { return &m_instance; }

    /*!
     * @brief 初期化
     * @param[in] pCallback ステータスバー更新用コールバック関数
    */
    void Initialize(void *pCallback);

    /*!
     * @brief 進捗表示コールバック関数の呼び出し
     * @param dRate     進捗率(0 - 1.0)
     * @param isInit    進捗率を入力値で初期化するか否か
    */
    void UpdateStatus(double dRate, bool isInit);

    /*!
     * @brief ステータスを初期状態にする
     * @param lodType 入力LOD種別
     * @param dLod3   lod3の場合の詳細度
    */
    void ResetStatus(CInputSettingData::LODType lodType);

    /*!
     * @brief 1工程単位の進捗率の取得
     * @return 1工程単位の進捗率
    */
    double GetRatePerProcess();

    /*!
     * @brief 次工程のステータスに遷移する
     * @note ResetStatus後は現在ステータス未設定状態のため、
             最初の工程の前にNextStatusを呼び出しステータスを遷移すること
    */
    void NextStatus();

    /*!
     * @brief   現在のステータス取得
     * @return  ステータス名
    */
    std::string NowStatus();


private:
    static CStatusManager m_instance;       // 自クラス唯一のインスタンス
    callback_type m_pCallback = nullptr;    // コールバック関数ポインタ
    std::queue<int> m_statusQueue;          // ステータス遷移スタック
    int m_nProcessCnt;                      // 工程数
    CInputSettingData::LODType m_lodType;   // 処理対象詳細度
    int m_nNowStatus;                       // 現在ステータス

    /*!
     * @brief コンストラクタ
    */
    CStatusManager();

    /*!
     * @brief デストラクタ
    */
    virtual ~CStatusManager();

    /*!
     * @brief   現在のステータス取得(LOD1用)
     * @return  ステータス名
    */
    std::string nowLod1Status();

    /*!
     * @brief   現在のステータス取得(LOD2用)
     * @return  ステータス名
    */
    std::string nowLod2Status();

    /*!
     * @brief   現在のステータス取得(LOD3用)
     * @return  ステータス名
    */
    std::string nowLod3Status();


};

