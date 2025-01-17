#pragma once
#include "CNetworkCreatorWrapper.h"
#include <locale.h>

using namespace NetworkCreatorWrapper;
using namespace System::Runtime::InteropServices;
using namespace System::Text;
/*!
 * @brief コンストラクタ
 * @param callback 進捗表示用コールバック関数
*/
CNetworkCreatorWrapper::CNetworkCreatorWrapper(ActionCallback ^callback)
{
    if (m_pCNCLib == nullptr)
    {
        m_pCallback = callback;
        m_innerCallback = gcnew ActionCallback(this, &CNetworkCreatorWrapper::callback);

        // 関数ポインタに変換
        System::IntPtr ptr = System::Runtime::InteropServices::Marshal::GetFunctionPointerForDelegate(m_innerCallback);
        // ライブラリ側にコールバック関数を伝達
        m_pCNCLib = new CNetworkCreatorLib((void *)ptr);
    }
}

/*!
 * @brief デストラクタ
*/
CNetworkCreatorWrapper::~CNetworkCreatorWrapper()
{
    this->!CNetworkCreatorWrapper();
}

/*!
 * @brief ファイナライザ
*/
CNetworkCreatorWrapper::!CNetworkCreatorWrapper()
{
    if (m_pCNCLib != nullptr)
    {
        delete m_pCNCLib;
        m_pCNCLib = nullptr;
    }
}

/*!
 * @brief 進捗表示用コールバック関数
 * @param dRate     進捗率
 * @param isInit    進捗率を入力値で初期化するか否か
*/
void CNetworkCreatorWrapper::callback(double dRate, bool isInit)
{
    m_pCallback(dRate, isInit);
}

/*!
 * @brief wchar->std::string変換
 * @param pInput 入力文字列(wcharのポインタ)
 * @return  std::stringの文字列
 * @note    入力文字数制限(1024)あり
*/
std::string CNetworkCreatorWrapper::wcharToString(const wchar_t *pInput)
{
    constexpr int CONV_STR_NUM = 1024;  // 変換字の制限文字数
    // windows環境のファイルパスの文字数制限は260文字のため、ファイルパス変換には耐えられるはず

    size_t convNum = 0;
    char chInput[CONV_STR_NUM] = "";
    _wsetlocale(LC_ALL, L"");
    errno_t err = wcstombs_s(&convNum, chInput, sizeof(chInput), pInput, _TRUNCATE);
    _wsetlocale(LC_ALL, L"C");
    std::string strDst = chInput;
    return strDst;
}

bool CNetworkCreatorWrapper::CreateNetwork(
    String ^strInputFolderPath,
    String ^strOutputFolderPath,
    int nLod,
    int nJPZone,
    bool bSHP,
    bool bGeoJSON)
{
    bool bRet = false;

    // 入出力フォルダパスの変換
    IntPtr pStrInput = Marshal::StringToHGlobalUni(strInputFolderPath);
    IntPtr pStrOutput = Marshal::StringToHGlobalUni(strOutputFolderPath);
    std::string strInput = wcharToString((wchar_t *)pStrInput.ToPointer());
    std::string strOutput = wcharToString((wchar_t *)pStrOutput.ToPointer());

    if (m_pCNCLib != nullptr)
    {
        m_pCNCLib->CreateNetwork(strInput, strOutput, nLod, nJPZone, bSHP, bGeoJSON);
    }

    Marshal::FreeHGlobal(pStrInput);
    Marshal::FreeHGlobal(pStrOutput);

    return bRet;
}