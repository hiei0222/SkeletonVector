using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Numerics;

namespace applim
{
    public static class vector
    {
        public static Vector3 ToVector3(this CameraSpacePoint point)
        {
            return new Vector3(point.X, point.Y, point.Z);
        }


    }
}
