#include "CCityGMLReader.h"
#include "CFileUtil.h"
#include "CUtil.h"

/*!
 * @brief 読み込み対象のCityGMLファイルの設定
 * @param strDirPath    入力フォルダパス
 * @return 読み込み対象の有無
 * @retval true     CityGML有り
 * @retval false    CityGML無し
*/
bool CCityGMLReader::Init(std::string strDirPath)
{
    bool bRet = false;
    ReleaseCityModels();
    m_cityGMLPathList.clear();

    if (CFileUtil::IsExistPath(strDirPath))
    {
        // gmlファイルの一覧を取得
        std::list<std::string> pathList;
        CFileUtil::CreateFileList(strDirPath, "gml", &pathList);
        for (std::string &strPath : pathList)
        {
            std::string strUtf8Path = CUtil::ConvShiftJisToUtf8(strPath);
            m_cityGMLPathList.push_back(strUtf8Path);
        }

        this->strInputDir = CUtil::ConvShiftJisToUtf8(strDirPath);
    }
    return (m_cityGMLPathList.size() > 0);
}

/*!
 * @brief CityGML読み込み(一括)
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CCityGMLReader::AllRead()
{
    bool bRet = false;
    ReleaseCityModels();

    while (GetTargetCityGMLFileNum() > 0)
    {
        // gml読み込み
        Read();
    }
    return (m_cityGMLObjects.size() > 0);
}

/*!
 * @brief CityGML読み込み(対象GMLの先頭1ファイルのみ)
 * @return 処理結果
 * @retval true     成功
 * @retval false    失敗
*/
bool CCityGMLReader::Read()
{
    bool bRet = false;
    if (!m_cityGMLPathList.empty())
    {
        // gml読み込み
        auto strName = m_cityGMLPathList.front();   // 先頭ファイル名を取得
        citygml::ParserParams params;
        params.tesselate = false;   // TINを作成しない
        std::shared_ptr<const citygml::CityModel> spCityModel = citygml::load(strName, params);
        if (spCityModel != nullptr)
        {
            bRet = true;
            m_cityModels.push_back(spCityModel);

            // CityObjectデータを取得
            for (const citygml::CityObject *pObj : spCityModel->getRootCityObjects())
            {
                m_cityGMLObjects.push_back(pObj);
            }
        }
        m_cityGMLPathList.pop_front();  // 先頭ファイル名を削除
    }
    return bRet;
}

/*!
 * @brief CityModel群のリリース
*/
void CCityGMLReader::ReleaseCityModels()
{
    m_cityGMLObjects.clear();

    if (m_cityModels.size() < 1)
    {
        return;
    }

    for (auto model : m_cityModels)
    {
        if (model)
        {
            model.reset();
        }
    }

    m_cityModels.clear();
}

/*!
 * @brief idによるCityObject検索
 * @param   strId   id
 * @return  検索結果
*/
const citygml::CityObject* CCityGMLReader::SearchCityObject(
    const std::string &strId)
{
    auto it = std::find_if(
        m_cityGMLObjects.begin(),
        m_cityGMLObjects.end(),
        [strId](const citygml::CityObject *ptr) {return ptr->getId() == strId; });

    if (it != m_cityGMLObjects.end())
        return *it;
    else
        return nullptr;
}
