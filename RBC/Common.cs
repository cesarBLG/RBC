using System;

namespace RBC
{
    public struct TrainPosition
    {
        public int LRBG;
        public float Distance;
        public float MaxSafe;
        public float MinSafe;
        public bool BackFacing;
    }
    enum Mode
    {
        FS,
        OS,
        SR,
        SH,
        UN,
        SL,
        SB,
        TR,
        PT,
        SF,
        IS,
        NL,
        LS,
        SN,
        RV,
        PS,
        NP
    }
    enum Level
    {
        L0,
        NTC,
        L1,
        L2,
        L3
    }
}