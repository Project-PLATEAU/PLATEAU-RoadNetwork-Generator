#pragma once
#include <string>
#include <list>
#include "CGeoUtil.h"
#include "CityGMLCommon.h"

#pragma region CityGML読み込み基底クラス
/*!
 * @brief CityGML読み込みクラス
*/
class CCityGMLReader
{
private:
    std::vector<std::shared_ptr<const citygml::CityModel>> m_cityModels;    // CityModel群
    std::list<std::string> m_cityGMLPathList;                   // 読み込み対象のファイルリスト
protected:
    std::vector<const citygml::CityObject *> m_cityGMLObjects;  // CityObjectデータのポインタ群

public:
    std::string strInputDir;  // 入力フォルダパス

    /*!
     * @brief コンストラクタ
    */
    CCityGMLReader() {};

    /*!
     * @brief デストラクタ
    */
    virtual ~CCityGMLReader()
    {
        ReleaseCityModels();
    };

    /*!
     * @brief 読み込み対象のCityGMLファイルの設定
     * @param strDirPath    入力フォルダパス
     * @return 読み込み対象の有無
     * @retval true     CityGML有り
     * @retval false    CityGML無し
    */
    bool Init(std::string strDirPath);

    /*!
     * @brief 読み込み対象ファイル数の取得
     * @return 読み込み対象ファイル数
    */
    size_t GetTargetCityGMLFileNum() { return m_cityGMLPathList.size(); }

    /*!
     * @brief CityGML読み込み(一括)
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool AllRead();

    /*!
     * @brief CityGML読み込み(対象GMLの先頭1ファイルのみ)
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool Read();

    /*!
     * @brief CityModel群のリリース
    */
    void ReleaseCityModels();

    /*!
     * @brief idによるCityObject検索
     * @param   strId   id
     * @return  検索結果
    */
    const citygml::CityObject *SearchCityObject(
        const std::string &strId);

    /*!
     * @brief   全てのCityObjectの取得
     * @return  CityObjectポインタの配列
    */
    std::vector<const citygml::CityObject *> GetCityObjects() { return m_cityGMLObjects; }
};
#pragma endregion
