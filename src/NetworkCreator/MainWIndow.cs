using System;
using System.Drawing;
using System.IO;
using System.Windows.Forms;
using System.Diagnostics;
using NetworkCreatorWrapper;

namespace NetworkCreator
{
    public partial class WinMain : Form
    {
        private const double m_PrgBarStatusMax = 100000.0;  // プログレスバーの最大値

        public WinMain()
        {
            InitializeComponent();

            // メインウィンドウのサイズ設定(サイズ変更不可)
            this.MaximizeBox = false;
            this.MaximumSize = this.Size;
            this.MinimumSize = this.Size;

            // 進捗表示用プログレスバーの設定
            PrgBarStatus.Minimum = 0;
            PrgBarStatus.Maximum = (int)m_PrgBarStatusMax;
            PrgBarStatus.Step = 1;
            PrgBarStatus.Value = 0;

            // 平面直角座標系設定
            for (int i = 1; i < 20; i++)
                CombBoxCoordinateSystem.Items.Add(i);
            CombBoxCoordinateSystem.SelectedIndex = 8;
        }

        /// <summary>
        /// 入力フォルダ選択ボタン押下時のイベント
        /// </summary>
        /// <param name="sender">オブジェクト</param>
        /// <param name="e">イベント変数</param>
        private void BtnInputFolder_Click(object sender, EventArgs e)
        {
            string strInputFolderPath = TxtBoxInputFolder.Text;
            if (!Directory.Exists(strInputFolderPath))
            {
                strInputFolderPath = @"C:\";
            }
            FolderBrowserDialog dlg = new FolderBrowserDialog
            {
                ShowNewFolderButton = false,        // 新規フォルダ作成はOFF
                SelectedPath = strInputFolderPath
            };
            DialogResult result =  dlg.ShowDialog();
            if (result == DialogResult.OK)
            {
                TxtBoxInputFolder.Text = dlg.SelectedPath;
            }
        }

        /// <summary>
        /// 出力フォルダ選択ボタン押下時のイベント
        /// </summary>
        /// <param name="sender">オブジェクト</param>
        /// <param name="e">イベント変数</param>
        private void BtnOutputFolder_Click(object sender, EventArgs e)
        {
            string strOutputFolderPath = TxtBoxOutputFolder.Text;
            if (!Directory.Exists(strOutputFolderPath))
            {
                strOutputFolderPath = @"C:\";
            }
            FolderBrowserDialog dlg = new FolderBrowserDialog
            {
                SelectedPath = strOutputFolderPath
            };
            DialogResult result = dlg.ShowDialog();
            if (result == DialogResult.OK)
            {
                TxtBoxOutputFolder.Text = dlg.SelectedPath;
            }
        }

        /// <summary>
        /// 作成ボタン押下時のイベント
        /// </summary>
        /// <param name="sender">オブジェクト</param>
        /// <param name="e">イベント変数</param>
        private void BtnCreate_Click(object sender, EventArgs e)
        {
            UIEnableControl(false);

            int nLod;
            if (RBtnLOD1.Checked)
                nLod = 1;
            else if (RBtnLOD2.Checked)
                nLod = 2;
            else if (RBtnLOD3.Checked)
                nLod = 3;
            else
                nLod = 0;

            // 平面直角座標系
            int nJPZone = CombBoxCoordinateSystem.SelectedIndex + 1;

            using (CNetworkCreatorWrapper cncWrapper = new CNetworkCreatorWrapper(UpdateStatus))
            {
                cncWrapper.CreateNetwork(TxtBoxInputFolder.Text, TxtBoxOutputFolder.Text, nLod, nJPZone, ChkShapefile.Checked, ChkGeoJSON.Checked);
            }

            MessageBox.Show("処理が終了しました", "メッセージ", MessageBoxButtons.OK, MessageBoxIcon.None);
            UIEnableControl(true);
            BtnEnd.Select();
        }

        /// <summary>
        /// 進捗プログレスバーの表示更新
        /// </summary>
        /// <param name="dRate">進捗率(0 - 1.0)</param>
        /// <param name="isInit">入力値で初期化するか否か</param>
        public void UpdateStatus(double dRate, bool isInit)
        {
            if (isInit)
            {
                // 入力値でプログレスバーの値を設定する
                PrgBarStatus.Value = (int)(dRate * m_PrgBarStatusMax);
            }
            else
            {
                // 入力値をプログレスバーの値に加算する
                PrgBarStatus.Value = PrgBarStatus.Value + (int)(dRate * m_PrgBarStatusMax);
            }
        }

        /// <summary>
        /// UIのEnable一括変更処理
        /// </summary>
        /// <param name="isEnable">有効フラグ</param>
        public void UIEnableControl(bool isEnable)
        {
            TxtBoxInputFolder.Enabled = isEnable;
            BtnInputFolder.Enabled = isEnable;
            RBtnLOD1.Enabled = isEnable;
            RBtnLOD2.Enabled = isEnable;
            RBtnLOD3.Enabled = isEnable;
            CombBoxCoordinateSystem.Enabled = isEnable;
            TxtBoxOutputFolder.Enabled = isEnable;
            BtnOutputFolder.Enabled = isEnable;
            ChkShapefile.Enabled = isEnable;
            ChkGeoJSON.Enabled = isEnable;
            BtnCreate.Enabled = isEnable;
            BtnEnd.Enabled = isEnable;
        }

        /// <summary>
        /// 終了ボタン押下時のイベント
        /// </summary>
        /// <param name="sender">オブジェクト</param>
        /// <param name="e">イベント変数</param>
        private void BtnEnd_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        /// <summary>
        /// マニュアルボタン押下時のイベント
        /// </summary>
        /// <param name="sender">オブジェクト</param>
        /// <param name="e">イベント変数</param>
        private void BtnManual_Click(object sender, EventArgs e)
        {
            String strManualPath = "./manual.pdf";

            if (System.IO.File.Exists(strManualPath))
            {
                ProcessStartInfo info = new ProcessStartInfo();
                info.FileName = "cmd";
                info.Arguments = @"/c start " + strManualPath;
                info.CreateNoWindow = true;         // ウィンドウを非表示
                info.UseShellExecute = false;
                Process p = Process.Start(info);    // 起動
                p.Close();
            }
            else
            {
                MessageBox.Show("アプリケーションと同階層にmanual.pdfが存在しません", "エラー", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }
    }
}
