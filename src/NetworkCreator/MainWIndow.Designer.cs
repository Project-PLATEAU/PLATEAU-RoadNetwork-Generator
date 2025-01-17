
namespace NetworkCreator
{
    partial class WinMain
    {
        /// <summary>
        /// 必要なデザイナー変数です。
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 使用中のリソースをすべてクリーンアップします。
        /// </summary>
        /// <param name="disposing">マネージド リソースを破棄する場合は true を指定し、その他の場合は false を指定します。</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows フォーム デザイナーで生成されたコード

        /// <summary>
        /// デザイナー サポートに必要なメソッドです。このメソッドの内容を
        /// コード エディターで変更しないでください。
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.LblOutputFolder = new System.Windows.Forms.Label();
            this.LblFileFormat = new System.Windows.Forms.Label();
            this.GrpBoxInput = new System.Windows.Forms.GroupBox();
            this.TlpInput = new System.Windows.Forms.TableLayoutPanel();
            this.LblInputRoadLOD = new System.Windows.Forms.Label();
            this.LblInputFolder = new System.Windows.Forms.Label();
            this.FlpRoadLOD = new System.Windows.Forms.FlowLayoutPanel();
            this.RBtnLOD1 = new System.Windows.Forms.RadioButton();
            this.RBtnLOD2 = new System.Windows.Forms.RadioButton();
            this.RBtnLOD3 = new System.Windows.Forms.RadioButton();
            this.LblLodDescription = new System.Windows.Forms.Label();
            this.TxtBoxInputFolder = new System.Windows.Forms.TextBox();
            this.BtnInputFolder = new System.Windows.Forms.Button();
            this.LblCoordinateSystem = new System.Windows.Forms.Label();
            this.CombBoxCoordinateSystem = new System.Windows.Forms.ComboBox();
            this.GrpBoxOutput = new System.Windows.Forms.GroupBox();
            this.TlpOutput = new System.Windows.Forms.TableLayoutPanel();
            this.TxtBoxOutputFolder = new System.Windows.Forms.TextBox();
            this.BtnOutputFolder = new System.Windows.Forms.Button();
            this.FlpFileFormat = new System.Windows.Forms.FlowLayoutPanel();
            this.ChkShapefile = new System.Windows.Forms.CheckBox();
            this.ChkGeoJSON = new System.Windows.Forms.CheckBox();
            this.BtnCreate = new System.Windows.Forms.Button();
            this.BtnEnd = new System.Windows.Forms.Button();
            this.TlpContents = new System.Windows.Forms.TableLayoutPanel();
            this.TblStatus = new System.Windows.Forms.TableLayoutPanel();
            this.LblStatus = new System.Windows.Forms.Label();
            this.PrgBarStatus = new System.Windows.Forms.ProgressBar();
            this.toolTipInputFolder = new System.Windows.Forms.ToolTip(this.components);
            this.BtnManual = new System.Windows.Forms.Button();
            this.GrpBoxInput.SuspendLayout();
            this.TlpInput.SuspendLayout();
            this.FlpRoadLOD.SuspendLayout();
            this.GrpBoxOutput.SuspendLayout();
            this.TlpOutput.SuspendLayout();
            this.FlpFileFormat.SuspendLayout();
            this.TlpContents.SuspendLayout();
            this.TblStatus.SuspendLayout();
            this.SuspendLayout();
            // 
            // LblOutputFolder
            // 
            this.LblOutputFolder.AutoSize = true;
            this.LblOutputFolder.Location = new System.Drawing.Point(5, 5);
            this.LblOutputFolder.Margin = new System.Windows.Forms.Padding(5);
            this.LblOutputFolder.Name = "LblOutputFolder";
            this.LblOutputFolder.Size = new System.Drawing.Size(64, 12);
            this.LblOutputFolder.TabIndex = 1;
            this.LblOutputFolder.Text = "出力フォルダ";
            // 
            // LblFileFormat
            // 
            this.LblFileFormat.AutoSize = true;
            this.LblFileFormat.Location = new System.Drawing.Point(5, 30);
            this.LblFileFormat.Margin = new System.Windows.Forms.Padding(5);
            this.LblFileFormat.Name = "LblFileFormat";
            this.LblFileFormat.Size = new System.Drawing.Size(63, 12);
            this.LblFileFormat.TabIndex = 3;
            this.LblFileFormat.Text = "ファイル形式";
            // 
            // GrpBoxInput
            // 
            this.GrpBoxInput.Controls.Add(this.TlpInput);
            this.GrpBoxInput.Location = new System.Drawing.Point(3, 3);
            this.GrpBoxInput.Name = "GrpBoxInput";
            this.GrpBoxInput.Size = new System.Drawing.Size(770, 101);
            this.GrpBoxInput.TabIndex = 4;
            this.GrpBoxInput.TabStop = false;
            this.GrpBoxInput.Text = "入力設定";
            // 
            // TlpInput
            // 
            this.TlpInput.ColumnCount = 3;
            this.TlpInput.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 15F));
            this.TlpInput.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 85F));
            this.TlpInput.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 70F));
            this.TlpInput.Controls.Add(this.LblInputRoadLOD, 0, 1);
            this.TlpInput.Controls.Add(this.LblInputFolder, 0, 0);
            this.TlpInput.Controls.Add(this.FlpRoadLOD, 1, 1);
            this.TlpInput.Controls.Add(this.TxtBoxInputFolder, 1, 0);
            this.TlpInput.Controls.Add(this.BtnInputFolder, 2, 0);
            this.TlpInput.Controls.Add(this.LblCoordinateSystem, 0, 2);
            this.TlpInput.Controls.Add(this.CombBoxCoordinateSystem, 1, 2);
            this.TlpInput.Location = new System.Drawing.Point(6, 13);
            this.TlpInput.Name = "TlpInput";
            this.TlpInput.RowCount = 3;
            this.TlpInput.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.TlpInput.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.TlpInput.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.TlpInput.Size = new System.Drawing.Size(758, 80);
            this.TlpInput.TabIndex = 8;
            // 
            // LblInputRoadLOD
            // 
            this.LblInputRoadLOD.AutoSize = true;
            this.LblInputRoadLOD.Location = new System.Drawing.Point(5, 30);
            this.LblInputRoadLOD.Margin = new System.Windows.Forms.Padding(5);
            this.LblInputRoadLOD.Name = "LblInputRoadLOD";
            this.LblInputRoadLOD.Size = new System.Drawing.Size(72, 15);
            this.LblInputRoadLOD.TabIndex = 2;
            this.LblInputRoadLOD.Text = "使用する道路LOD";
            // 
            // LblInputFolder
            // 
            this.LblInputFolder.AutoSize = true;
            this.LblInputFolder.Location = new System.Drawing.Point(5, 5);
            this.LblInputFolder.Margin = new System.Windows.Forms.Padding(5);
            this.LblInputFolder.Name = "LblInputFolder";
            this.LblInputFolder.Size = new System.Drawing.Size(64, 12);
            this.LblInputFolder.TabIndex = 0;
            this.LblInputFolder.Text = "入力フォルダ";
            // 
            // FlpRoadLOD
            // 
            this.FlpRoadLOD.Controls.Add(this.RBtnLOD1);
            this.FlpRoadLOD.Controls.Add(this.RBtnLOD2);
            this.FlpRoadLOD.Controls.Add(this.RBtnLOD3);
            this.FlpRoadLOD.Controls.Add(this.LblLodDescription);
            this.FlpRoadLOD.Dock = System.Windows.Forms.DockStyle.Fill;
            this.FlpRoadLOD.Location = new System.Drawing.Point(103, 25);
            this.FlpRoadLOD.Margin = new System.Windows.Forms.Padding(0);
            this.FlpRoadLOD.Name = "FlpRoadLOD";
            this.FlpRoadLOD.Size = new System.Drawing.Size(584, 25);
            this.FlpRoadLOD.TabIndex = 2;
            // 
            // RBtnLOD1
            // 
            this.RBtnLOD1.AutoSize = true;
            this.RBtnLOD1.Checked = true;
            this.RBtnLOD1.Location = new System.Drawing.Point(3, 3);
            this.RBtnLOD1.Name = "RBtnLOD1";
            this.RBtnLOD1.Size = new System.Drawing.Size(51, 16);
            this.RBtnLOD1.TabIndex = 3;
            this.RBtnLOD1.TabStop = true;
            this.RBtnLOD1.Text = "LOD1";
            this.RBtnLOD1.UseVisualStyleBackColor = true;
            // 
            // RBtnLOD2
            // 
            this.RBtnLOD2.AutoSize = true;
            this.RBtnLOD2.Location = new System.Drawing.Point(60, 3);
            this.RBtnLOD2.Name = "RBtnLOD2";
            this.RBtnLOD2.Size = new System.Drawing.Size(51, 16);
            this.RBtnLOD2.TabIndex = 4;
            this.RBtnLOD2.Text = "LOD2";
            this.RBtnLOD2.UseVisualStyleBackColor = true;
            // 
            // RBtnLOD3
            // 
            this.RBtnLOD3.AutoSize = true;
            this.RBtnLOD3.Location = new System.Drawing.Point(117, 3);
            this.RBtnLOD3.Name = "RBtnLOD3";
            this.RBtnLOD3.Size = new System.Drawing.Size(51, 16);
            this.RBtnLOD3.TabIndex = 5;
            this.RBtnLOD3.Text = "LOD3";
            this.RBtnLOD3.UseVisualStyleBackColor = true;
            // 
            // LblLodDescription
            // 
            this.LblLodDescription.AutoSize = true;
            this.LblLodDescription.Location = new System.Drawing.Point(174, 3);
            this.LblLodDescription.Margin = new System.Windows.Forms.Padding(3);
            this.LblLodDescription.Name = "LblLodDescription";
            this.LblLodDescription.Padding = new System.Windows.Forms.Padding(0, 1, 0, 0);
            this.LblLodDescription.Size = new System.Drawing.Size(372, 13);
            this.LblLodDescription.TabIndex = 6;
            this.LblLodDescription.Text = "※LOD1の場合は車道のみ、LOD2,3の場合は車歩道のネットワークを作成する";
            // 
            // TxtBoxInputFolder
            // 
            this.TxtBoxInputFolder.Location = new System.Drawing.Point(106, 3);
            this.TxtBoxInputFolder.Name = "TxtBoxInputFolder";
            this.TxtBoxInputFolder.Size = new System.Drawing.Size(578, 19);
            this.TxtBoxInputFolder.TabIndex = 0;
            this.toolTipInputFolder.SetToolTip(this.TxtBoxInputFolder, "CityGMLを格納しているルートフォルダパスを指定する(例:C:\\13999_tokyo_mlit_2023_citygml_1_op)");
            // 
            // BtnInputFolder
            // 
            this.BtnInputFolder.Location = new System.Drawing.Point(690, 3);
            this.BtnInputFolder.Name = "BtnInputFolder";
            this.BtnInputFolder.Size = new System.Drawing.Size(55, 19);
            this.BtnInputFolder.TabIndex = 1;
            this.BtnInputFolder.Text = "選択";
            this.BtnInputFolder.UseVisualStyleBackColor = true;
            this.BtnInputFolder.Click += new System.EventHandler(this.BtnInputFolder_Click);
            // 
            // LblCoordinateSystem
            // 
            this.LblCoordinateSystem.AutoSize = true;
            this.LblCoordinateSystem.Location = new System.Drawing.Point(5, 55);
            this.LblCoordinateSystem.Margin = new System.Windows.Forms.Padding(5);
            this.LblCoordinateSystem.Name = "LblCoordinateSystem";
            this.LblCoordinateSystem.Size = new System.Drawing.Size(89, 12);
            this.LblCoordinateSystem.TabIndex = 15;
            this.LblCoordinateSystem.Text = "平面直角座標系";
            // 
            // CombBoxCoordinateSystem
            // 
            this.CombBoxCoordinateSystem.FormattingEnabled = true;
            this.CombBoxCoordinateSystem.Location = new System.Drawing.Point(106, 53);
            this.CombBoxCoordinateSystem.Name = "CombBoxCoordinateSystem";
            this.CombBoxCoordinateSystem.Size = new System.Drawing.Size(60, 20);
            this.CombBoxCoordinateSystem.TabIndex = 6;
            // 
            // GrpBoxOutput
            // 
            this.GrpBoxOutput.Controls.Add(this.TlpOutput);
            this.GrpBoxOutput.Location = new System.Drawing.Point(3, 110);
            this.GrpBoxOutput.Name = "GrpBoxOutput";
            this.GrpBoxOutput.Size = new System.Drawing.Size(770, 78);
            this.GrpBoxOutput.TabIndex = 5;
            this.GrpBoxOutput.TabStop = false;
            this.GrpBoxOutput.Text = "出力設定";
            // 
            // TlpOutput
            // 
            this.TlpOutput.ColumnCount = 3;
            this.TlpOutput.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 15F));
            this.TlpOutput.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 85F));
            this.TlpOutput.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 70F));
            this.TlpOutput.Controls.Add(this.TxtBoxOutputFolder, 1, 0);
            this.TlpOutput.Controls.Add(this.BtnOutputFolder, 2, 0);
            this.TlpOutput.Controls.Add(this.LblOutputFolder, 0, 0);
            this.TlpOutput.Controls.Add(this.LblFileFormat, 0, 1);
            this.TlpOutput.Controls.Add(this.FlpFileFormat, 1, 1);
            this.TlpOutput.Location = new System.Drawing.Point(6, 18);
            this.TlpOutput.Name = "TlpOutput";
            this.TlpOutput.RowCount = 2;
            this.TlpOutput.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.TlpOutput.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.TlpOutput.Size = new System.Drawing.Size(758, 53);
            this.TlpOutput.TabIndex = 6;
            // 
            // TxtBoxOutputFolder
            // 
            this.TxtBoxOutputFolder.Location = new System.Drawing.Point(106, 3);
            this.TxtBoxOutputFolder.Name = "TxtBoxOutputFolder";
            this.TxtBoxOutputFolder.Size = new System.Drawing.Size(578, 19);
            this.TxtBoxOutputFolder.TabIndex = 7;
            // 
            // BtnOutputFolder
            // 
            this.BtnOutputFolder.Location = new System.Drawing.Point(690, 3);
            this.BtnOutputFolder.Name = "BtnOutputFolder";
            this.BtnOutputFolder.Size = new System.Drawing.Size(56, 19);
            this.BtnOutputFolder.TabIndex = 8;
            this.BtnOutputFolder.Text = "選択";
            this.BtnOutputFolder.UseVisualStyleBackColor = true;
            this.BtnOutputFolder.Click += new System.EventHandler(this.BtnOutputFolder_Click);
            // 
            // FlpFileFormat
            // 
            this.FlpFileFormat.Controls.Add(this.ChkShapefile);
            this.FlpFileFormat.Controls.Add(this.ChkGeoJSON);
            this.FlpFileFormat.Dock = System.Windows.Forms.DockStyle.Fill;
            this.FlpFileFormat.Location = new System.Drawing.Point(106, 28);
            this.FlpFileFormat.Name = "FlpFileFormat";
            this.FlpFileFormat.Size = new System.Drawing.Size(578, 22);
            this.FlpFileFormat.TabIndex = 9;
            // 
            // ChkShapefile
            // 
            this.ChkShapefile.AutoSize = true;
            this.ChkShapefile.Checked = true;
            this.ChkShapefile.CheckState = System.Windows.Forms.CheckState.Checked;
            this.ChkShapefile.Location = new System.Drawing.Point(3, 3);
            this.ChkShapefile.Name = "ChkShapefile";
            this.ChkShapefile.Size = new System.Drawing.Size(71, 16);
            this.ChkShapefile.TabIndex = 10;
            this.ChkShapefile.Text = "Shapefile";
            this.ChkShapefile.UseVisualStyleBackColor = true;
            // 
            // ChkGeoJSON
            // 
            this.ChkGeoJSON.AutoSize = true;
            this.ChkGeoJSON.Checked = true;
            this.ChkGeoJSON.CheckState = System.Windows.Forms.CheckState.Checked;
            this.ChkGeoJSON.Location = new System.Drawing.Point(80, 3);
            this.ChkGeoJSON.Name = "ChkGeoJSON";
            this.ChkGeoJSON.Size = new System.Drawing.Size(74, 16);
            this.ChkGeoJSON.TabIndex = 11;
            this.ChkGeoJSON.Text = "GeoJSON";
            this.ChkGeoJSON.UseVisualStyleBackColor = true;
            // 
            // BtnCreate
            // 
            this.BtnCreate.Location = new System.Drawing.Point(618, 276);
            this.BtnCreate.Name = "BtnCreate";
            this.BtnCreate.Size = new System.Drawing.Size(75, 23);
            this.BtnCreate.TabIndex = 12;
            this.BtnCreate.Text = "作成";
            this.BtnCreate.UseVisualStyleBackColor = true;
            this.BtnCreate.Click += new System.EventHandler(this.BtnCreate_Click);
            // 
            // BtnEnd
            // 
            this.BtnEnd.Location = new System.Drawing.Point(713, 276);
            this.BtnEnd.Name = "BtnEnd";
            this.BtnEnd.Size = new System.Drawing.Size(75, 23);
            this.BtnEnd.TabIndex = 13;
            this.BtnEnd.Text = "終了";
            this.BtnEnd.UseVisualStyleBackColor = true;
            this.BtnEnd.Click += new System.EventHandler(this.BtnEnd_Click);
            // 
            // TlpContents
            // 
            this.TlpContents.ColumnCount = 1;
            this.TlpContents.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.TlpContents.Controls.Add(this.GrpBoxInput, 0, 0);
            this.TlpContents.Controls.Add(this.GrpBoxOutput, 0, 1);
            this.TlpContents.Controls.Add(this.TblStatus, 0, 2);
            this.TlpContents.Location = new System.Drawing.Point(12, 12);
            this.TlpContents.Name = "TlpContents";
            this.TlpContents.RowCount = 3;
            this.TlpContents.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.TlpContents.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.TlpContents.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.TlpContents.Size = new System.Drawing.Size(776, 237);
            this.TlpContents.TabIndex = 9;
            // 
            // TblStatus
            // 
            this.TblStatus.ColumnCount = 2;
            this.TblStatus.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 13.96574F));
            this.TblStatus.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 86.03426F));
            this.TblStatus.Controls.Add(this.LblStatus, 0, 0);
            this.TblStatus.Controls.Add(this.PrgBarStatus, 1, 0);
            this.TblStatus.Location = new System.Drawing.Point(8, 196);
            this.TblStatus.Margin = new System.Windows.Forms.Padding(8, 5, 8, 5);
            this.TblStatus.Name = "TblStatus";
            this.TblStatus.RowCount = 2;
            this.TblStatus.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.TblStatus.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.TblStatus.Size = new System.Drawing.Size(759, 36);
            this.TblStatus.TabIndex = 6;
            // 
            // LblStatus
            // 
            this.LblStatus.AutoSize = true;
            this.LblStatus.Location = new System.Drawing.Point(5, 5);
            this.LblStatus.Margin = new System.Windows.Forms.Padding(5);
            this.LblStatus.Name = "LblStatus";
            this.LblStatus.Size = new System.Drawing.Size(50, 12);
            this.LblStatus.TabIndex = 2;
            this.LblStatus.Text = "ステータス";
            // 
            // PrgBarStatus
            // 
            this.PrgBarStatus.Location = new System.Drawing.Point(108, 3);
            this.PrgBarStatus.Name = "PrgBarStatus";
            this.PrgBarStatus.Size = new System.Drawing.Size(645, 19);
            this.PrgBarStatus.TabIndex = 3;
            // 
            // BtnManual
            // 
            this.BtnManual.Location = new System.Drawing.Point(12, 276);
            this.BtnManual.Name = "BtnManual";
            this.BtnManual.Size = new System.Drawing.Size(33, 23);
            this.BtnManual.TabIndex = 14;
            this.BtnManual.Text = "?";
            this.BtnManual.UseVisualStyleBackColor = true;
            this.BtnManual.Click += new System.EventHandler(this.BtnManual_Click);
            // 
            // WinMain
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(800, 311);
            this.Controls.Add(this.BtnManual);
            this.Controls.Add(this.BtnEnd);
            this.Controls.Add(this.BtnCreate);
            this.Controls.Add(this.TlpContents);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedSingle;
            this.Name = "WinMain";
            this.Text = "ネットワークデータ作成支援ツール";
            this.GrpBoxInput.ResumeLayout(false);
            this.TlpInput.ResumeLayout(false);
            this.TlpInput.PerformLayout();
            this.FlpRoadLOD.ResumeLayout(false);
            this.FlpRoadLOD.PerformLayout();
            this.GrpBoxOutput.ResumeLayout(false);
            this.TlpOutput.ResumeLayout(false);
            this.TlpOutput.PerformLayout();
            this.FlpFileFormat.ResumeLayout(false);
            this.FlpFileFormat.PerformLayout();
            this.TlpContents.ResumeLayout(false);
            this.TblStatus.ResumeLayout(false);
            this.TblStatus.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion
        private System.Windows.Forms.Label LblOutputFolder;
        private System.Windows.Forms.Label LblFileFormat;
        private System.Windows.Forms.GroupBox GrpBoxInput;
        private System.Windows.Forms.GroupBox GrpBoxOutput;
        private System.Windows.Forms.CheckBox ChkGeoJSON;
        private System.Windows.Forms.CheckBox ChkShapefile;
        private System.Windows.Forms.Button BtnCreate;
        private System.Windows.Forms.Button BtnEnd;
        private System.Windows.Forms.FlowLayoutPanel FlpRoadLOD;
        private System.Windows.Forms.RadioButton RBtnLOD1;
        private System.Windows.Forms.RadioButton RBtnLOD2;
        private System.Windows.Forms.RadioButton RBtnLOD3;
        private System.Windows.Forms.Label LblInputFolder;
        private System.Windows.Forms.Label LblInputRoadLOD;
        private System.Windows.Forms.TableLayoutPanel TlpInput;
        private System.Windows.Forms.TextBox TxtBoxInputFolder;
        private System.Windows.Forms.TableLayoutPanel TlpOutput;
        private System.Windows.Forms.FlowLayoutPanel FlpFileFormat;
        private System.Windows.Forms.TableLayoutPanel TlpContents;
        private System.Windows.Forms.Button BtnOutputFolder;
        private System.Windows.Forms.Button BtnInputFolder;
        private System.Windows.Forms.TextBox TxtBoxOutputFolder;
        private System.Windows.Forms.TableLayoutPanel TblStatus;
        private System.Windows.Forms.Label LblStatus;
        private System.Windows.Forms.ProgressBar PrgBarStatus;
        private System.Windows.Forms.Label LblCoordinateSystem;
        private System.Windows.Forms.ComboBox CombBoxCoordinateSystem;
        private System.Windows.Forms.Label LblLodDescription;
        private System.Windows.Forms.ToolTip toolTipInputFolder;
        private System.Windows.Forms.Button BtnManual;
    }
}

