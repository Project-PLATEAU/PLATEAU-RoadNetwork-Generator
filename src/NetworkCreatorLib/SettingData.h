#pragma once
#include <string>

/*!
 * @brief 入力パス設定クラス
*/
class CInputSettingData
{
private:
    static CInputSettingData m_instance;    // 自クラス唯一のインスタンス
    /*!
     * @brief コンストラクタ
    */
    CInputSettingData();

    /*!
     * @brief デストラクタ
    */
    virtual ~CInputSettingData();

public:
    /*!
     * @brief LOD種別
    */
    enum class LODType
    {
        UNKNOWN = -1,   // 未設定
        LOD1 = 1,       // LOD1
        LOD2,           // LOD2
        LOD3            // LOD3
    };

    std::string strInputFolderPath; // 入力フォルダのルートパス
    std::string strTranFolderPath;  // 交通(道路)フォルダパス
    std::string strFrnFolderPath;   // 都市設備フォルダパス
    std::string strBridFolderPath;  // 橋梁フォルダパス
    LODType lodType;                // 使用するLOD種別
    int nJPZone;                    // 平面直角座標系

    /*!
     * @brief インスタンスの取得
     * @return 入力設定インスタンス
    */
    static CInputSettingData *GetInstance() { return &m_instance; }

    /*!
     * @brief 初期化(入力フォルダパスの設定)
     * @param strPath   入力フォルダのルートパス
     * @param nLod      使用するLOD
     * @param nJPZone   平面直角座標系
    */
    void Initialize(std::string strPath, int nLod, int nJPZone);
};

/*!
 * @brief 出力パス設定クラス
*/
class COutputSettingData
{
private:
    static COutputSettingData m_instance;    // 自クラス唯一のインスタンス

    /*!
     * @brief コンストラクタ
    */
    COutputSettingData();

    /*!
     * @brief デストラクタ
    */
    virtual ~COutputSettingData();
public:
    std::string strOutputFolderPath;            // 出力フォルダのルートパス
    std::string strResultFolderPath;            // 結果フォルダ(出力フォルダのルートパス下の日付フォルダ)
    std::string strShpRoadwayFolderPath;        // 車道のSHPフォルダパス
    std::string strShpFootpathFolderPath;       // 歩道のSHPフォルダパス
    std::string strGeoJsonRoadwayFolderPath;    // 車道のGeoJSONフォルダパス
    std::string strGeoJsonFootpathFolderPath;   // 歩道のGeoJSONフォルダパス
    std::string strLogFilePath;                 // ログファイルパス
    std::string strRoadwayErrLogFilePath;       // 車道用エラーログファイルパス
    std::string strFootpathErrLogFilePath;      // 歩道用エラーログファイルパス
    bool bCreateSHP;                            // SHP作成フラグ
    bool bCreateGeoJSON;                        // GeoJSON作成フラグ

    /*!
     * @brief インスタンスの取得
     * @return 入力設定インスタンス
    */
    static COutputSettingData *GetInstance() { return &m_instance; }

    /*!
     * @brief 初期化(出力フォルダパスの設定とフォルダ作成)
     * @param strPath   出力フォルダのルートパス
     * @param bSHP      SHP作成フラグ
     * @param bGeoJSON  GeoJSON作成フラグ
     * @param nLod      使用する道路のLOD
    */
    void Initialize(std::string strPath, bool bSHP, bool bGeoJSON, int nLod);
};
