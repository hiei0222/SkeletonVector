using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;
using System.Numerics;

namespace applim
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    /// 

    

    public partial class MainWindow : Window
    {

        KinectSensor kinect;
        BodyFrameReader bodyFrameReader;
        Body[] bodies;


        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            try
            {
                // open kinect
                kinect = KinectSensor.GetDefault();
                kinect.Open();
                // open body reader
                bodyFrameReader = kinect.BodyFrameSource.OpenReader();
                
                bodyFrameReader.FrameArrived += bodyFrameReader_FrameArrived;

                // put body data into array
                bodies = new Body[kinect.BodyFrameSource.BodyCount];

                


            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
                Close();
            }
        }

        void bodyFrameReader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            UpdateBodyFrame(e);
            DrawBodyFrame();

        }
        
        
        private void DrawBodyFrame()
        {
            CanvasBody.Children.Clear();

            foreach (var body in bodies.Where(b => b.IsTracked))
            {
                // Distance
                var right = body.Joints[JointType.HandRight];
                var left = body.Joints[JointType.HandLeft];
                if ((right.TrackingState == TrackingState.Tracked) &&
                        (left.TrackingState == TrackingState.Tracked))
                {

                    // meters to centimeters
                    var distance = Vector3.Distance(
                        right.Position.ToVector3(),
                        left.Position.ToVector3())
                        * 100;
                    DistanceMessage.Text = distance.ToString();
                }

                // Angel
                var hand = body.Joints[JointType.HandRight];
                var elbow = body.Joints[JointType.ElbowRight];
                var shoulder = body.Joints[JointType.ShoulderRight];

                var v1 = hand.Position.ToVector3() - elbow.Position.ToVector3();
                var v2 = shoulder.Position.ToVector3() - elbow.Position.ToVector3();

                const float Rad2Deg = 180.0f / (float)Math.PI;
                var dot = Math.Acos(Vector3.Dot(v1, v2) / (v1.Length() * v2.Length()));
                var angle = (float)(dot * Rad2Deg);
                AngelMessage.Text = angle.ToString();

                foreach (var joint in body.Joints)
                {
                    if (joint.Value.TrackingState == TrackingState.Tracked)
                    {
                        DrawEllipse(joint.Value, 10, Brushes.Blue);
                        
                    }
                    else if (joint.Value.TrackingState == TrackingState.Inferred)
                    {
                        DrawEllipse(joint.Value, 10, Brushes.Yellow);
                    }
                }
            }
        }
        


        
        private void DrawEllipse(Joint joint, int p, SolidColorBrush solidColorBrush)
        {
            var ellipse = new Ellipse()
            {
                Width = p,
                Height = p,
                Fill = solidColorBrush,
            };

            var point = kinect.CoordinateMapper.MapCameraPointToDepthSpace(joint.Position);
            if ((point.X < 0) || (point.Y < 0))
            {
                return;
            }

            Canvas.SetLeft(ellipse, point.X - (p / 2));
            Canvas.SetTop(ellipse, point.Y - (p / 2));

            CanvasBody.Children.Add(ellipse);
        }

        // renew body data
        private void UpdateBodyFrame(BodyFrameArrivedEventArgs e)
        {
            using (var bodyFrame = e.FrameReference.AcquireFrame()) {
                if (bodyFrame == null)
                {
                    return;
                }
                // get bodies data
                bodyFrame.GetAndRefreshBodyData(bodies);
            }
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (bodyFrameReader != null)
            {
                bodyFrameReader.Dispose();
                bodyFrameReader = null;
            }
            if (kinect != null)
            {
                kinect.Close();
                kinect = null;
            }
        }
    }
}
