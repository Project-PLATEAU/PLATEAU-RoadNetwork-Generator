#pragma once
#include <string>
#include "SettingData.h"

/*!
 * @brief ネットワークデータ作成ライブラリクラス
*/
class CNetworkCreatorLib
{
private:
    /*!
     * @brief 設定情報ログ出力
    */
    void writeSettingLog();

public:
    /*!
     * @brief コンストラクタ
     * @param pCallback 進捗表示コールバック関数ポインタ
    */
    CNetworkCreatorLib(void *pCallback);


    /*!
     * @brief デストラクタ
    */
    ~CNetworkCreatorLib();


    bool CreateNetwork(
        const std::string &strInputDir,
        const std::string &strOutputDir,
        int nLod,
        int nJPZone,
        bool bSHP,
        bool bGeoJSON);
};
