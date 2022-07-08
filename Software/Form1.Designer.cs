
namespace PID_Control
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.bt_connect = new System.Windows.Forms.Button();
            this.cbb_Select_Com = new System.Windows.Forms.ComboBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.lb_status = new System.Windows.Forms.Label();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.bt_clear = new System.Windows.Forms.Button();
            this.bt_send = new System.Windows.Forms.Button();
            this.bt_reset = new System.Windows.Forms.Button();
            this.bt_stop = new System.Windows.Forms.Button();
            this.bt_start = new System.Windows.Forms.Button();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.txt_setpoint = new System.Windows.Forms.TextBox();
            this.txt_kd = new System.Windows.Forms.TextBox();
            this.txt_ki = new System.Windows.Forms.TextBox();
            this.txt_kp = new System.Windows.Forms.TextBox();
            this.zedGraphControl1 = new ZedGraph.ZedGraphControl();
            this.COM = new System.IO.Ports.SerialPort(this.components);
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.txt_receive = new System.Windows.Forms.TextBox();
            this.groupBox5 = new System.Windows.Forms.GroupBox();
            this.txt_send = new System.Windows.Forms.TextBox();
            this.GetPPR = new System.Windows.Forms.GroupBox();
            this.label6 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.txt_ppr = new System.Windows.Forms.TextBox();
            this.txt_rpm = new System.Windows.Forms.TextBox();
            this.btnGetPPR = new System.Windows.Forms.Button();
            this.groupBox6 = new System.Windows.Forms.GroupBox();
            this.btnEncx4 = new System.Windows.Forms.Button();
            this.btnEncx1 = new System.Windows.Forms.Button();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.groupBox3.SuspendLayout();
            this.groupBox4.SuspendLayout();
            this.groupBox5.SuspendLayout();
            this.GetPPR.SuspendLayout();
            this.groupBox6.SuspendLayout();
            this.SuspendLayout();
            // 
            // bt_connect
            // 
            this.bt_connect.BackColor = System.Drawing.SystemColors.Control;
            this.bt_connect.ForeColor = System.Drawing.Color.Lime;
            this.bt_connect.Location = new System.Drawing.Point(172, 43);
            this.bt_connect.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.bt_connect.Name = "bt_connect";
            this.bt_connect.Size = new System.Drawing.Size(109, 39);
            this.bt_connect.TabIndex = 0;
            this.bt_connect.Text = "Connect";
            this.bt_connect.UseVisualStyleBackColor = false;
            this.bt_connect.Click += new System.EventHandler(this.bt_connect_Click);
            // 
            // cbb_Select_Com
            // 
            this.cbb_Select_Com.FormattingEnabled = true;
            this.cbb_Select_Com.Location = new System.Drawing.Point(21, 43);
            this.cbb_Select_Com.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.cbb_Select_Com.Name = "cbb_Select_Com";
            this.cbb_Select_Com.Size = new System.Drawing.Size(135, 24);
            this.cbb_Select_Com.TabIndex = 1;
            // 
            // groupBox1
            // 
            this.groupBox1.BackColor = System.Drawing.SystemColors.GradientActiveCaption;
            this.groupBox1.Controls.Add(this.lb_status);
            this.groupBox1.Controls.Add(this.cbb_Select_Com);
            this.groupBox1.Controls.Add(this.bt_connect);
            this.groupBox1.Location = new System.Drawing.Point(12, 12);
            this.groupBox1.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox1.Size = new System.Drawing.Size(287, 123);
            this.groupBox1.TabIndex = 2;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "CONNECT";
            // 
            // lb_status
            // 
            this.lb_status.AutoSize = true;
            this.lb_status.Location = new System.Drawing.Point(21, 100);
            this.lb_status.Name = "lb_status";
            this.lb_status.Size = new System.Drawing.Size(141, 17);
            this.lb_status.TabIndex = 2;
            this.lb_status.Text = "STATUS: Disconnect";
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.bt_clear);
            this.groupBox2.Controls.Add(this.bt_send);
            this.groupBox2.Controls.Add(this.bt_reset);
            this.groupBox2.Controls.Add(this.bt_stop);
            this.groupBox2.Controls.Add(this.bt_start);
            this.groupBox2.Location = new System.Drawing.Point(304, 12);
            this.groupBox2.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox2.Size = new System.Drawing.Size(376, 144);
            this.groupBox2.TabIndex = 3;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "SYSTEM CONTROL";
            // 
            // bt_clear
            // 
            this.bt_clear.Location = new System.Drawing.Point(211, 100);
            this.bt_clear.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.bt_clear.Name = "bt_clear";
            this.bt_clear.Size = new System.Drawing.Size(147, 30);
            this.bt_clear.TabIndex = 11;
            this.bt_clear.Text = "Clear data";
            this.bt_clear.UseVisualStyleBackColor = true;
            this.bt_clear.Click += new System.EventHandler(this.bt_clear_Click);
            // 
            // bt_send
            // 
            this.bt_send.Location = new System.Drawing.Point(27, 100);
            this.bt_send.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.bt_send.Name = "bt_send";
            this.bt_send.Size = new System.Drawing.Size(147, 30);
            this.bt_send.TabIndex = 10;
            this.bt_send.Text = "Send PID";
            this.bt_send.UseVisualStyleBackColor = true;
            this.bt_send.Click += new System.EventHandler(this.bt_send_Click);
            // 
            // bt_reset
            // 
            this.bt_reset.Location = new System.Drawing.Point(281, 43);
            this.bt_reset.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.bt_reset.Name = "bt_reset";
            this.bt_reset.Size = new System.Drawing.Size(75, 39);
            this.bt_reset.TabIndex = 2;
            this.bt_reset.Text = "RESET";
            this.bt_reset.UseVisualStyleBackColor = true;
            this.bt_reset.Click += new System.EventHandler(this.bt_reset_Click);
            // 
            // bt_stop
            // 
            this.bt_stop.Location = new System.Drawing.Point(155, 39);
            this.bt_stop.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.bt_stop.Name = "bt_stop";
            this.bt_stop.Size = new System.Drawing.Size(81, 39);
            this.bt_stop.TabIndex = 1;
            this.bt_stop.Text = "STOP";
            this.bt_stop.UseVisualStyleBackColor = true;
            this.bt_stop.Click += new System.EventHandler(this.bt_stop_Click);
            // 
            // bt_start
            // 
            this.bt_start.Location = new System.Drawing.Point(27, 41);
            this.bt_start.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.bt_start.Name = "bt_start";
            this.bt_start.Size = new System.Drawing.Size(75, 39);
            this.bt_start.TabIndex = 0;
            this.bt_start.Text = "START";
            this.bt_start.UseVisualStyleBackColor = true;
            this.bt_start.Click += new System.EventHandler(this.bt_start_Click);
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.label4);
            this.groupBox3.Controls.Add(this.label3);
            this.groupBox3.Controls.Add(this.label2);
            this.groupBox3.Controls.Add(this.label1);
            this.groupBox3.Controls.Add(this.txt_setpoint);
            this.groupBox3.Controls.Add(this.txt_kd);
            this.groupBox3.Controls.Add(this.txt_ki);
            this.groupBox3.Controls.Add(this.txt_kp);
            this.groupBox3.Location = new System.Drawing.Point(685, 17);
            this.groupBox3.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox3.Size = new System.Drawing.Size(349, 118);
            this.groupBox3.TabIndex = 4;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "PID PARAMETERS";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(169, 76);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(60, 17);
            this.label4.TabIndex = 7;
            this.label4.Text = "Setpoint";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(173, 30);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(20, 17);
            this.label3.TabIndex = 6;
            this.label3.Text = "KI";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(7, 76);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(27, 17);
            this.label2.TabIndex = 5;
            this.label2.Text = "KD";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(7, 30);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(26, 17);
            this.label1.TabIndex = 4;
            this.label1.Text = "KP";
            // 
            // txt_setpoint
            // 
            this.txt_setpoint.Location = new System.Drawing.Point(244, 76);
            this.txt_setpoint.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.txt_setpoint.Name = "txt_setpoint";
            this.txt_setpoint.Size = new System.Drawing.Size(100, 22);
            this.txt_setpoint.TabIndex = 3;
            // 
            // txt_kd
            // 
            this.txt_kd.Location = new System.Drawing.Point(59, 76);
            this.txt_kd.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.txt_kd.Name = "txt_kd";
            this.txt_kd.Size = new System.Drawing.Size(100, 22);
            this.txt_kd.TabIndex = 2;
            // 
            // txt_ki
            // 
            this.txt_ki.Location = new System.Drawing.Point(244, 30);
            this.txt_ki.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.txt_ki.Name = "txt_ki";
            this.txt_ki.Size = new System.Drawing.Size(100, 22);
            this.txt_ki.TabIndex = 1;
            // 
            // txt_kp
            // 
            this.txt_kp.Location = new System.Drawing.Point(59, 30);
            this.txt_kp.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.txt_kp.Name = "txt_kp";
            this.txt_kp.Size = new System.Drawing.Size(100, 22);
            this.txt_kp.TabIndex = 0;
            // 
            // zedGraphControl1
            // 
            this.zedGraphControl1.AllowDrop = true;
            this.zedGraphControl1.Location = new System.Drawing.Point(13, 166);
            this.zedGraphControl1.Margin = new System.Windows.Forms.Padding(5, 5, 5, 5);
            this.zedGraphControl1.Name = "zedGraphControl1";
            this.zedGraphControl1.ScrollGrace = 0D;
            this.zedGraphControl1.ScrollMaxX = 0D;
            this.zedGraphControl1.ScrollMaxY = 0D;
            this.zedGraphControl1.ScrollMaxY2 = 0D;
            this.zedGraphControl1.ScrollMinX = 0D;
            this.zedGraphControl1.ScrollMinY = 0D;
            this.zedGraphControl1.ScrollMinY2 = 0D;
            this.zedGraphControl1.Size = new System.Drawing.Size(909, 490);
            this.zedGraphControl1.TabIndex = 5;
            // 
            // COM
            // 
            this.COM.BaudRate = 115200;
            this.COM.PortName = "Serial";
            this.COM.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(this.OnCom);
            // 
            // timer1
            // 
            this.timer1.Enabled = true;
            this.timer1.Interval = 30;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.txt_receive);
            this.groupBox4.Location = new System.Drawing.Point(931, 260);
            this.groupBox4.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox4.Size = new System.Drawing.Size(228, 208);
            this.groupBox4.TabIndex = 6;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "Receive Data";
            // 
            // txt_receive
            // 
            this.txt_receive.AllowDrop = true;
            this.txt_receive.Location = new System.Drawing.Point(11, 34);
            this.txt_receive.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.txt_receive.Multiline = true;
            this.txt_receive.Name = "txt_receive";
            this.txt_receive.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.txt_receive.Size = new System.Drawing.Size(211, 179);
            this.txt_receive.TabIndex = 0;
            // 
            // groupBox5
            // 
            this.groupBox5.Controls.Add(this.txt_send);
            this.groupBox5.Location = new System.Drawing.Point(931, 473);
            this.groupBox5.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox5.Name = "groupBox5";
            this.groupBox5.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox5.Size = new System.Drawing.Size(228, 185);
            this.groupBox5.TabIndex = 7;
            this.groupBox5.TabStop = false;
            this.groupBox5.Text = "Send Data";
            // 
            // txt_send
            // 
            this.txt_send.Location = new System.Drawing.Point(11, 21);
            this.txt_send.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.txt_send.Multiline = true;
            this.txt_send.Name = "txt_send";
            this.txt_send.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.txt_send.Size = new System.Drawing.Size(212, 181);
            this.txt_send.TabIndex = 0;
            // 
            // GetPPR
            // 
            this.GetPPR.Controls.Add(this.label6);
            this.GetPPR.Controls.Add(this.label5);
            this.GetPPR.Controls.Add(this.txt_ppr);
            this.GetPPR.Controls.Add(this.txt_rpm);
            this.GetPPR.Controls.Add(this.btnGetPPR);
            this.GetPPR.Location = new System.Drawing.Point(937, 148);
            this.GetPPR.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.GetPPR.Name = "GetPPR";
            this.GetPPR.Padding = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.GetPPR.Size = new System.Drawing.Size(221, 106);
            this.GetPPR.TabIndex = 8;
            this.GetPPR.TabStop = false;
            this.GetPPR.Text = "Get PPR";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(72, 70);
            this.label6.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(36, 17);
            this.label6.TabIndex = 4;
            this.label6.Text = "PPR";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(72, 17);
            this.label5.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(38, 17);
            this.label5.TabIndex = 3;
            this.label5.Text = "RPM";
            // 
            // txt_ppr
            // 
            this.txt_ppr.Location = new System.Drawing.Point(119, 66);
            this.txt_ppr.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.txt_ppr.Name = "txt_ppr";
            this.txt_ppr.Size = new System.Drawing.Size(87, 22);
            this.txt_ppr.TabIndex = 2;
            // 
            // txt_rpm
            // 
            this.txt_rpm.Location = new System.Drawing.Point(119, 14);
            this.txt_rpm.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.txt_rpm.Name = "txt_rpm";
            this.txt_rpm.Size = new System.Drawing.Size(87, 22);
            this.txt_rpm.TabIndex = 1;
            // 
            // btnGetPPR
            // 
            this.btnGetPPR.Location = new System.Drawing.Point(8, 38);
            this.btnGetPPR.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.btnGetPPR.Name = "btnGetPPR";
            this.btnGetPPR.Size = new System.Drawing.Size(100, 28);
            this.btnGetPPR.TabIndex = 0;
            this.btnGetPPR.Text = "GetPPR";
            this.btnGetPPR.UseVisualStyleBackColor = true;
            this.btnGetPPR.Click += new System.EventHandler(this.btnGetPPR_Click);
            // 
            // groupBox6
            // 
            this.groupBox6.Controls.Add(this.btnEncx4);
            this.groupBox6.Controls.Add(this.btnEncx1);
            this.groupBox6.Location = new System.Drawing.Point(1056, 17);
            this.groupBox6.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.groupBox6.Name = "groupBox6";
            this.groupBox6.Padding = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.groupBox6.Size = new System.Drawing.Size(103, 123);
            this.groupBox6.TabIndex = 9;
            this.groupBox6.TabStop = false;
            this.groupBox6.Text = "Encoder";
            // 
            // btnEncx4
            // 
            this.btnEncx4.Location = new System.Drawing.Point(3, 70);
            this.btnEncx4.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.btnEncx4.Name = "btnEncx4";
            this.btnEncx4.Size = new System.Drawing.Size(95, 41);
            this.btnEncx4.TabIndex = 1;
            this.btnEncx4.Text = "Mode x4";
            this.btnEncx4.UseVisualStyleBackColor = true;
            this.btnEncx4.Click += new System.EventHandler(this.btnEncx4_Click);
            // 
            // btnEncx1
            // 
            this.btnEncx1.Location = new System.Drawing.Point(3, 23);
            this.btnEncx1.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.btnEncx1.Name = "btnEncx1";
            this.btnEncx1.Size = new System.Drawing.Size(95, 41);
            this.btnEncx1.TabIndex = 0;
            this.btnEncx1.Text = "Mode x1";
            this.btnEncx1.UseVisualStyleBackColor = true;
            this.btnEncx1.Click += new System.EventHandler(this.btnEncx1_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.SystemColors.GradientActiveCaption;
            this.ClientSize = new System.Drawing.Size(1167, 671);
            this.Controls.Add(this.groupBox6);
            this.Controls.Add(this.GetPPR);
            this.Controls.Add(this.groupBox5);
            this.Controls.Add(this.groupBox4);
            this.Controls.Add(this.zedGraphControl1);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.Name = "Form1";
            this.Text = "PID CONTROL";
            this.Load += new System.EventHandler(this.Form1_Load);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            this.groupBox4.ResumeLayout(false);
            this.groupBox4.PerformLayout();
            this.groupBox5.ResumeLayout(false);
            this.groupBox5.PerformLayout();
            this.GetPPR.ResumeLayout(false);
            this.GetPPR.PerformLayout();
            this.groupBox6.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Button bt_connect;
        private System.Windows.Forms.ComboBox cbb_Select_Com;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.Button bt_reset;
        private System.Windows.Forms.Button bt_stop;
        private System.Windows.Forms.Button bt_start;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox txt_setpoint;
        private System.Windows.Forms.TextBox txt_kd;
        private System.Windows.Forms.TextBox txt_ki;
        private System.Windows.Forms.TextBox txt_kp;
        private ZedGraph.ZedGraphControl zedGraphControl1;
        private System.IO.Ports.SerialPort COM;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.Label lb_status;
        private System.Windows.Forms.Button bt_send;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.TextBox txt_receive;
        private System.Windows.Forms.GroupBox groupBox5;
        private System.Windows.Forms.TextBox txt_send;
        private System.Windows.Forms.Button bt_clear;
        private System.Windows.Forms.GroupBox GetPPR;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.TextBox txt_ppr;
        private System.Windows.Forms.TextBox txt_rpm;
        private System.Windows.Forms.Button btnGetPPR;
        private System.Windows.Forms.GroupBox groupBox6;
        private System.Windows.Forms.Button btnEncx4;
        private System.Windows.Forms.Button btnEncx1;
    }
}

