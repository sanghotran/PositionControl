using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;
using ZedGraph;
using System.Threading;

namespace PID_Control
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
            //Control.CheckForIllegalCrossThreadCalls = false;
        }
        // -------------------Event---------------------------------------
        private void Form1_Load(object sender, EventArgs e)
        {
            bt_clear.Enabled = false;
            bt_reset.Enabled = false;
            bt_send.Enabled = false;
            bt_start.Enabled = false;
            bt_stop.Enabled = false;
            btnEncx1.Enabled = false;
            btnEncx4.Enabled = false;
            btnGetPPR.Enabled = false;
            txt_kp.Enabled = false;
            txt_ki.Enabled = false;
            txt_kd.Enabled = false;
            txt_setpoint.Enabled = false;
            txt_send.Enabled = false;
            txt_receive.Enabled = false;
            txt_ppr.Enabled = false;
            txt_rpm.Enabled = false;
            // creat Graph           
            InitGraph();


        }

        private void bt_connect_Click(object sender, EventArgs e)
        {
            if (bt_connect.Text == "Connect")
            {
                COM.PortName = cbb_Select_Com.Text;
                COM.Open();
                bt_connect.Text = "Disconnect";
                bt_connect.ForeColor = Color.Red;
                lb_status.Text = "STATUS: Connect";
                txt_receive.Text += "Serial port " + cbb_Select_Com.Text + " opened " + " \n";
                bt_clear.Enabled = true;
                bt_reset.Enabled = true;
                bt_send.Enabled = true;
                bt_start.Enabled = true;
                bt_stop.Enabled = true;
                btnEncx1.Enabled = true;
                btnEncx4.Enabled = true;
                btnGetPPR.Enabled = true;
                txt_kp.Enabled = true;
                txt_ki.Enabled = true;
                txt_kd.Enabled = true;
                txt_setpoint.Enabled = true;
                txt_send.Enabled = true;
                txt_receive.Enabled = true;
                txt_ppr.Enabled = true;
                txt_rpm.Enabled = true;
            }
            else if (bt_connect.Text == "Disconnect")
            {
                COM.Close();
                bt_connect.Text = "Connect";
                bt_connect.ForeColor = Color.Lime;
                lb_status.Text = "STATUS: Disconnect";
                txt_receive.Text += "Serial port " + cbb_Select_Com.Text + " closed \n";
                bt_clear.Enabled = false;
                bt_reset.Enabled = false;
                bt_send.Enabled = false;
                bt_start.Enabled = false;
                bt_stop.Enabled = false;
                btnEncx1.Enabled = false;
                btnEncx4.Enabled = false;
                btnGetPPR.Enabled = false;
                txt_kp.Enabled = false;
                txt_ki.Enabled = false;
                txt_kd.Enabled = false;
                txt_setpoint.Enabled = false;
                txt_send.Enabled = false;
                txt_receive.Enabled = false;
                flag_enc_mode = false;
                txt_ppr.Enabled = false;
                txt_rpm.Enabled = false;
                btnEncx4.BackColor = System.Drawing.SystemColors.GradientActiveCaption;
                btnEncx1.BackColor = System.Drawing.SystemColors.GradientActiveCaption;
            }
        }
        private bool flag_enc_mode = false;
        private void bt_start_Click(object sender, EventArgs e)
        {
            txbuff = "0 0 1 \n";
            if (flag_enc_mode == false)
            {
                txt_send.Text += "Please Choose mode" + Environment.NewLine;
            }
            else
            COM.Write(txbuff);
        }

        private void bt_stop_Click(object sender, EventArgs e)
        {
            txbuff = "0 1 0 \n";
            COM.Write(txbuff);

        }

        private void bt_reset_Click(object sender, EventArgs e)
        {
            txbuff = "0 2 0 \n";
            flag_enc_mode = false;
            btnEncx4.BackColor = System.Drawing.SystemColors.GradientActiveCaption;
            btnEncx1.BackColor = System.Drawing.SystemColors.GradientActiveCaption;
            COM.Write(txbuff);
        }
        private void btnGetPPR_Click(object sender, EventArgs e)
        {
            txbuff = "2" + " " + txt_rpm.Text + " " + txt_ppr.Text + "  2" + " \n";
            if (flag_enc_mode == false)
            {
                txt_send.Text += "Please Choose mode" + Environment.NewLine;
            }
            else
            COM.Write(txbuff);
        }
        private void btnEncx1_Click(object sender, EventArgs e)
        {
            txbuff = "3 1 0 \n";
            flag_enc_mode = true;
            btnEncx1.BackColor = Color.DarkCyan;
            btnEncx4.BackColor = System.Drawing.SystemColors.GradientActiveCaption;
            COM.Write(txbuff);
        }
        private void btnEncx4_Click(object sender, EventArgs e)
        {
            txbuff = "3 4 0 \n";
            flag_enc_mode = true;
            btnEncx4.BackColor = Color.DarkCyan;
            btnEncx1.BackColor = System.Drawing.SystemColors.GradientActiveCaption;
            COM.Write(txbuff);
        }
        private void bt_send_Click(object sender, EventArgs e)
        {
            txbuff = "1" + " " + txt_kp.Text + " " + txt_ki.Text + " " + txt_kd.Text + " " + txt_setpoint.Text + " \n";
            COM.Write(txbuff);
            flag_setpoint = false;

        }


        int intlen = 0;
        float t_samp = 0.03F;
        private void timer1_Tick(object sender, EventArgs e)
        {
            string[] ports = SerialPort.GetPortNames();
            if (intlen != ports.Length)
            {
                intlen = ports.Length;
                cbb_Select_Com.Items.Clear();
                for (int j = 0; j < intlen; j++)
                {
                    cbb_Select_Com.Items.Add(ports[j]);
                }

            }
            //test graph
            //realtime += 0.03;
            //position += 50;
            //DrawGraph();

        }


        string txbuff;
        string rxbuff;
        
        double a = 0;
        bool flag_setpoint = false;
        private void processrxbuffer()//object sender, EventArgs e)
        {
            try
            {
                rxdata = rxbuff.Split(' ');
                if (rxdata[0] == "0")
                {

                    position = Convert.ToDouble(rxdata[1]);
                    u_control = Convert.ToDouble(rxdata[2]);

                    //txt_receive.Text += "\n U Control is " + rxdata[2] + " \n";
                    //txt_receive.Text += "\n Position is " + rxdata[1] + " \n";
                    realtime += 0.03;
                    //position += 50;
                    DrawGraph();
                    if (flag_setpoint == false)
                    {
                        //if (position < 0)
                        //{
                        //    position = position * (-1);
                        //}
                        if (((Math.Abs(position) > 0.8 * Math.Abs((Convert.ToDouble(txt_setpoint.Text)))) && (Math.Abs(position) < 1.2 * Math.Abs((Convert.ToDouble(txt_setpoint.Text))))) || (Convert.ToDouble(txt_setpoint.Text) == 0))

                        {
                            if ((a - position) == 0)
                            {
                                txt_receive.Text += "Position is " + rxdata[1] + Environment.NewLine;
                                flag_setpoint = true;
                                a = 0;
                            }

                            else a = position;
                        }
                       //else { txt_receive.Text += "ERROR out of range.."+ Environment.NewLine; }
                    }


                }
                // rxdata[0] == "1" de xac nhan mode trong truong hop can phat trien sau nay
                else if (rxdata[0] == "1")
                {
                    txt_receive.Text += "\n PPR is " + rxdata[1] + "\n";
                    txt_ppr.Text = rxdata[1];
                }
                else if (rxdata[0] == "2")
                {
                    txt_send.Text += "Parameter KP is " + rxdata[1] + " \n";
                    txt_send.Text += "Parameter KI is " + rxdata[2] + " \n";
                    txt_send.Text += "Parameter KD is " + rxdata[3] + " \n";
                    txt_send.Text += "Parameter Setpoint is " + rxdata[4] + " \n";

                }
                else if (rxdata[0] == "3")
                {
                    txt_receive.Text += "Getting PPR ..." + Environment.NewLine;
                }
                else if (rxdata[0] == "4")
                {
                    txt_receive.Text += "Encoder Mode x1" + Environment.NewLine;
                }
                else if (rxdata[0] == "5")
                {
                    txt_receive.Text += "Encoder Mode x4" + Environment.NewLine;
                }
                //rxdata[4] + '\n'; 
            }
            catch (Exception)
            {
            }
        }

        private void OnCom(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            try
            {
                rxbuff = COM.ReadLine();
                //Thread thread = new Thread(processrxbuffer);
                //thread.IsBackground = true;
                //thread.Start();

                //Invoke(new Action(() =>
                //{
                //    processrxbuffer();
                //}));

                //processrxbuffer();

                txt_receive.Invoke(new MethodInvoker(processrxbuffer));
            }
            catch (Exception)
            { }
        }
        string[] rxdata;
        double position, u_control;
        private void bt_clear_Click(object sender, EventArgs e)
        {
            txt_receive.Text = "";
            txt_send.Text = "";
            realtime = 0;
            ClearGraph();
        }

        

        //--------------------Func----------------------

        private void InitGraph()
        {
            zedGraphControl1.GraphPane.Title.Text = "Đồ thị hiển thị đáp ứng";
            zedGraphControl1.GraphPane.XAxis.Title.Text = "Time(s)";
            zedGraphControl1.GraphPane.YAxis.Title.Text = "Position";
            RollingPointPairList list = new RollingPointPairList(99999);
            LineItem curve = zedGraphControl1.GraphPane.AddCurve("Position", list, Color.Red, SymbolType.None);
            zedGraphControl1.GraphPane.XAxis.Scale.Min = 0;
            zedGraphControl1.GraphPane.XAxis.Scale.Max = 30;
            zedGraphControl1.GraphPane.XAxis.Scale.MinorStep = 0.03;
            zedGraphControl1.GraphPane.XAxis.Scale.MajorStep = 3;
            zedGraphControl1.GraphPane.YAxis.Scale.Min = -1000;
            zedGraphControl1.GraphPane.YAxis.Scale.Max = 1000;
            zedGraphControl1.GraphPane.AxisChange();
        }
        private void DrawGraph()
        {

            try
            {
                LineItem curve = zedGraphControl1.GraphPane.CurveList[0] as LineItem;

                IPointListEdit list = curve.Points as IPointListEdit;
                list.Add(realtime, position);

                Scale xScale = zedGraphControl1.GraphPane.XAxis.Scale;
                Scale yScale = zedGraphControl1.GraphPane.YAxis.Scale;

                // Tự động Scale theo trục x
                if (realtime > xScale.Max - xScale.MajorStep)
                {
                    xScale.Max = realtime + xScale.MajorStep;
                    xScale.Min = xScale.Max - 30;
                }

                // Tự động Scale theo trục y
                if (position > yScale.Max - yScale.MajorStep)
                {
                    yScale.Max = position + yScale.MajorStep;
                }
                else if (position < yScale.Min + yScale.MajorStep)
                {
                    yScale.Min = position - yScale.MajorStep;
                }
                zedGraphControl1.AxisChange();
                zedGraphControl1.Invalidate();
                zedGraphControl1.Refresh();
            }
            catch (Exception)
            {
            }
        }

        private void ClearGraph()
        {
            zedGraphControl1.GraphPane.CurveList.Clear();
            zedGraphControl1.GraphPane.GraphObjList.Clear();
            zedGraphControl1.AxisChange();
            zedGraphControl1.Invalidate();
            InitGraph();
        }

        





        //------------Properties------------------------
        private double realtime = 0;


    }
}
