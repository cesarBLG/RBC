// COPYRIGHT 2024 by the Open Rails project.
//
// This file is part of Open Rails.
//
// Open Rails is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Open Rails is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Open Rails.  If not, see <http://www.gnu.org/licenses/>.
using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using Orts.Formats.Msts;
using Orts.MultiPlayer;
using Orts.Simulation;
using Orts.Simulation.Signalling;
using Orts.Simulation.Physics;
using ORTS.Common;
namespace RBC
{
    public class ETCSVariables
    {
        List<bool> bits;
        public int Offset;
        public int Length => bits.Count;
        public ETCSVariables()
        {
            bits = new List<bool>();
        }
        public ETCSVariables(byte[] buffer)
        {
            bits = new List<bool>(buffer.Length*8);
            for (int i=0; i<buffer.Length; i++)
            {
                for (int j=0; j<8; j++)
                {
                    bits.Add(((buffer[i] >> (7 - j)) & 1) == 1);
                }
            }
        }
        public void Push(long variable, int size)
        {
            for (int i=0; i<size; i++)
            {
                bits.Add(((variable>>(size-i-1)) & 1) == 1);
            }
        }
        public void Replace(int pos, long variable, int size)
        {
            for (int i = 0; i < size; i++)
            {
                bits[pos + i] = ((variable >> (size - i - 1)) & 1) == 1;
            }
        }
        public long Access(int pos, int size)
        {
            long v = 0;
            for (int i=0; i<size; i++)
            {
                if (bits[pos + Offset + i]) v |= 1 << (size - i - 1);
            }
            return v;
        }
        public long Read(int size)
        {
            long l = Access(0, size);
            Offset += size;
            return l;
        }
        public float ReadDistanceM(int scale)
        {
            return Read(15) * (scale == 2 ? 10.0f : (scale == 0 ? 0.1f : 1.0f));
        }
        public float ReadSpeedMpS()
        {
            return Read(7)*5/3.6f;
        }
        public void Append(ETCSVariables vars)
        {
            bits.AddRange(vars.bits);
        }
        public byte[] ToArray()
        {
            int size = (bits.Count + 7) / 8;
            byte[] data = new byte[size];
            int div = bits.Count / 8;
            int rem = bits.Count % 8;
            for (int i = 0; i < div; i++)
            {
                byte c = 0;
                for (int j = 0; j < 8; j++)
                {
                    if (bits[8*i+j]) c |= (byte)(1 << (7 - j));
                }
                data[i] = c;
            }
            if (rem > 0)
            {
                byte c = 0;
                for (int i = 0; i < rem; i++)
                {
                    if (bits[8 * div + i]) c |= (byte)(1 << (7 - i));
                }
                data[div] = c;
            }
            data[1] = (byte)(size >> 2);
            data[2] = (byte)((size << 6) | (data[2] & 63));
            return data;
        }
    }
    public class RBC
    {
        List<RBC_connection> Connections = new List<RBC_connection>();
        Task<Socket> ListenTask = null;
        Socket Listener;
        public RBC(int NID_C, int NID_RBC, int port=0x7911)
        {
            Signals signals = MPManager.Simulator.Signals;
            /*var flags = BindingFlags.Instance | BindingFlags.NonPublic;
            var field = typeof(CsSignalScripts).GetField("Simulator", flags);*/
            RBC_session.Signals = signals;
            
            RBC_session.NID_C = NID_C;
            RBC_session.NID_RBC = NID_RBC;

            string network = "ADIF";
            if (network == "ADIF")
            {
                RBC_session.AllowOSbeforeFS = false;
                RBC_session.AllowTAF = false;
                RBC_session.L2TransitionAnnouncementWithMA = true;
                RBC_session.L2TransitionOrder = true;
                RBC_session.MABeforeTransition = true;
            }
            else
            {
                RBC_session.AllowOSbeforeFS = true;
                RBC_session.AllowTAF = false;
                RBC_session.L2TransitionAnnouncementWithMA = true;
                RBC_session.L2TransitionOrder = false;
                RBC_session.MABeforeTransition = true;
            }
            LoadParameter<bool>("RBC","OsBeforeFs", ref RBC_session.AllowOSbeforeFS);
            LoadParameter<bool>("RBC","TAF", ref RBC_session.AllowTAF);
            LoadParameter<bool>("RBC","ERTMSRoutes", ref RBC_session.ERTMSRoutes);
            
            IPEndPoint localEndPoint = new IPEndPoint(IPAddress.Any, port);
            Listener = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            Listener.Bind(localEndPoint);
            Listener.Listen(10);
        }
        private void LoadParameter<T>(string sectionName, string keyName, ref T value)
        {
            string file = Path.Combine(MPManager.Simulator.RoutePath, @"rbc.ini");
            if (File.Exists(file))
            {
                string buffer = new string('\0', 256);
                int length = NativeMethods.GetPrivateProfileString(sectionName, keyName, null, buffer, buffer.Length, file);

                if (length > 0)
                {
                    buffer.Trim();
                    value = (T)Convert.ChangeType(buffer, typeof(T), System.Globalization.CultureInfo.InvariantCulture);
                }
            }
        }
        public void Update()
        {
            if (RBC_session.Signals == null)
            {
                RBC_session.Signals = MPManager.Simulator.Signals;
            }

            try
            {
                if (ListenTask == null || ListenTask.IsCompleted)
                {
                    if (ListenTask != null)
                    {
                        Connections.Add(new RBC_connection(ListenTask.Result));
                    }
                    ListenTask = Listener.AcceptAsync();
                }

                List<int> oldSessions = new List<int>();
                foreach (var kvp in RBC_connection.sessions)
                {
                    if (!kvp.Value.Established && !kvp.Value.Establishing) oldSessions.Add(kvp.Key);
                }
                foreach (int id in oldSessions)
                {
                    RBC_connection.sessions.Remove(id);
                }
                Connections.RemoveAll(x => 
                {
                    if (!x.Active)
                    {
                        x.Dispose();
                        return true;
                    }
                    return false;
                });
                foreach (var connection in Connections)
                {
                    connection.Update();
                }
                foreach (var session in RBC_connection.sessions)
                {
                    session.Value.Update();
                }
            }
            catch (Exception e)
            {
                Console.WriteLine("==============RBC error==============");
                Console.WriteLine(e.StackTrace);
                Console.WriteLine("==============RBC error==============");
            }
        }
    }
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
    public class RBC_session
    {
        public bool Establishing;
        public bool Established;
        public int NID_ENGINE;
        public RBC_connection Connection;
        DateTime LocalReferenceTime;
        uint? ReferenceTimeStamp;
        uint? LastTrainTimeStamp;
        uint LastTimeStamp;
        TrainPosition? position;
        public Train.TCSubpathRoute TrainRoute = new Train.TCSubpathRoute();
        public Train.TCPosition LRBGPosition;
        public Train.TCPosition TrainPosition;
        Mode CurrentMode = Mode.NP;
        Level CurrentLevel;
        float SpeedMpS;
        public static Signals Signals;
        Dictionary<uint, (ETCSVariables,DateTime,int)> PendingAck = new Dictionary<uint, (ETCSVariables, DateTime, int)>();
        Dictionary<int, int> PositionReportIndex = new Dictionary<int, int>();
        RouteInfo? SentInfo;
        (int signal, Aspecto aspecto, DateTime time) NextSignalCleared;
        int EmergencyStopCount = 0;
        bool TrainDataAcknowledged = false;
        Level LevelTrAnnouncement = Level.NTC;
        Dictionary<int, SignalHead> EmergencyStops = new Dictionary<int, SignalHead>();
        Dictionary<int, bool> LevelCrossings = new Dictionary<int, bool>();

        public static bool AllowTAF = false;
        public static bool AllowOSbeforeFS = false;
        public static bool L2TransitionAnnouncementWithMA = false;
        public static bool L2TransitionOrder = true;
        public static bool MABeforeTransition = false;
        public static bool ERTMSRoutes = false;
        public static int NID_C;
        public static int NID_RBC;
        List<ETCSVariables> OptionalPackets = new List<ETCSVariables>();
        public RBC_session(RBC_connection con, int nid_engine)
        {
            Establishing = true;
            textoAAspecto = Enum.GetNames(typeof(Aspecto)).ToDictionary(x => x, x => (Aspecto)Enum.Parse(typeof(Aspecto), x), StringComparer.OrdinalIgnoreCase);
            PositionReportIndex[129] = 74;
            PositionReportIndex[130] = 74;
            PositionReportIndex[132] = 79;
            PositionReportIndex[136] = 74;
            PositionReportIndex[137] = 106;
            PositionReportIndex[138] = 106;
            PositionReportIndex[147] = 80;
            PositionReportIndex[149] = 74;
            PositionReportIndex[150] = 74;
            PositionReportIndex[157] = 76;
            PositionReportIndex[158] = 82;
            Connection = con;
            NID_ENGINE = nid_engine;
        }
        public void Handle(ETCSVariables msg)
        {
            int nid_message = (int)msg.Access(0, 8);
            uint t_train = (uint)msg.Access(18, 32);
            int nid_engine = (int)msg.Access(50, 24);

            if (LastTrainTimeStamp != null && t_train - LastTrainTimeStamp > uint.MaxValue / 2)
                return; // Reject if message is older than previously received one
            LastTrainTimeStamp = t_train; // Store T_TRAIN of last message
            var now = DateTime.UtcNow;
            if (ReferenceTimeStamp == null) ReferenceTimeStamp = t_train; // Set T_TRAIN as our origin of times if not already set
            else ReferenceTimeStamp += (uint)(now.Subtract(LocalReferenceTime).TotalMilliseconds/10); // Advance time with measured elapsed time (prevent radio delays)
            if (t_train - ReferenceTimeStamp > 6000 && ReferenceTimeStamp - t_train > 6000)
            {
                // Clocks out of sync!!!
                TerminateSession();
                return;
            }
            if (t_train - ReferenceTimeStamp < 3000) ReferenceTimeStamp = t_train; // Advance our origin of times so it is always in advance of T_TRAIN
            LocalReferenceTime = now;
            uint stamp = ReferenceTimeStamp.Value;
            if (LastTimeStamp - stamp < 20) LastTimeStamp++;
            else LastTimeStamp = stamp;

            if (PositionReportIndex.ContainsKey(nid_message))
            {
                msg.Offset = PositionReportIndex[nid_message];
                Level prevL = CurrentLevel;
                Mode prevM = CurrentMode;
                var prevPosition = position;
                position = ReadPositionReport(msg);
                BuildTrainRoute();
                if (LRBGPosition != null && (prevPosition == null || prevPosition?.LRBG != position?.LRBG)) SendLinking();
                if (position != null && prevPosition == null)
                {
                    // Revocation of L1 TSRs
                    var revok = new ETCSVariables();
                    revok.Push(64, 8);
                    revok.Push(2, 2);
                    revok.Push(23, 13);
                    OptionalPackets.Add(revok);
                    /*var nation = new ETCSVariables();
                    nation.Push(3, 8);
                    nation.Push(1, 2);
                    nation.Push(0, 13);
                    nation.Push(1, 2);
                    nation.Push(32767, 15);
                    nation.Push(352, 10);
                    nation.Push(0, 5);
                    nation.Push(6, 7);
                    nation.Push(20, 7);
                    nation.Push(6, 7);
                    nation.Push(20, 7);
                    nation.Push(40, 7);
                    nation.Push(6, 7);
                    nation.Replace(10, nation.Length, 13);
                    OptionalPackets.Add(nation);*/
                }
                if ((prevL == Level.L2 && CurrentLevel != Level.L2) || CurrentMode == Mode.SH || CurrentMode == Mode.SL)
                {
                    TerminateSession();
                    return;
                }
                if (prevM != Mode.SB && CurrentMode == Mode.SB && nid_message != 157) TerminateSession();
                if (prevL != CurrentLevel) LevelTrAnnouncement = CurrentLevel;
                if (prevM != Mode.PT && CurrentMode == Mode.PT)
                {
                    // Acknowledgement of exit from trip mode
                    var ack = new ETCSVariables();
                    ack.Push(6, 8);
                    ack.Push(0, 10);
                    ack.Push(0, 32);
                    ack.Push(1, 1);
                    ack.Push(position?.LRBG ?? 16777215, 24);
                    SendMessage(ack, true);
                }
                if (prevM != Mode.FS && prevM != Mode.OS && (CurrentMode == Mode.FS || CurrentMode == Mode.OS))
                {
                    // MA request parameters
                    var marq = new ETCSVariables();
                    marq.Push(57, 8);
                    marq.Push(2, 2);
                    marq.Push(49, 13);
                    marq.Push(40, 8); // 40s before perturb location
                    marq.Push(40, 10); // 40s before timer expires
                    marq.Push(20, 8); // Repeat every 20s
                    OptionalPackets.Add(marq);
                }
                if (prevM != CurrentMode && (CurrentMode != Mode.FS && CurrentMode != Mode.OS)) LevelCrossings.Clear();
                msg.Offset = 0;
            }
            switch (nid_message)
            {
                case 155: // Initiation of communication session
                    // Send system version
                    Established = false;
                    Establishing = true;
                    var version = new ETCSVariables();
                    version.Push(32, 8);
                    version.Push(0, 10);
                    version.Push(0, 32);
                    version.Push(1, 1);
                    version.Push(position?.LRBG ?? 16777215, 24);
                    version.Push(33, 7);
                    SendMessage(version, true);
                    break;
                case 156: // Termination of communication session
                    // Send ack of termination
                    var end = new ETCSVariables();
                    end.Push(39, 8);
                    end.Push(0, 10);
                    end.Push(0, 32);
                    end.Push(0, 1);
                    end.Push(position?.LRBG ?? 16777215, 24);
                    Established = false;
                    SendMessage(end, false);
                    Connection = null;
                    break;
                case 129: // Validated train data
                    // Send ack of train data
                    var ack = new ETCSVariables();
                    ack.Push(8, 8);
                    ack.Push(0, 10);
                    ack.Push(0, 32);
                    ack.Push(1, 1);
                    ack.Push(position?.LRBG ?? 16777215, 24);
                    ack.Push(t_train, 32);
                    SendMessage(ack, true);
                    TrainDataAcknowledged = true;
                    break;
                case 159: // Session established
                    Established = true;
                    Establishing = false;
                    break;
                case 147: // Ack of emergency stop
                    msg.Offset = 74;
                    int id = (int)msg.Read(4);
                    int stopAccepted = (int)msg.Read(2);
                    if (EmergencyStops.ContainsKey(id))
                    {
                        if (stopAccepted != 3)
                        {
                            if (NextSignalCleared.signal == EmergencyStops[id].mainSignal.thisRef) NextSignalCleared.signal = -1;
                            if (SentInfo != null)
                            {
                                var info = SentInfo.Value;
                                info.EndSignal = EmergencyStops[id];
                                for (int i=0; i<info.Signals.Count-1; i++)
                                {
                                    if (info.Signals[i].Item1 == info.EndSignal)
                                    {
                                        info.Signals.RemoveRange(i+1, info.Signals.Count - i - 1);
                                        break;
                                    }
                                }
                                SentInfo = info;
                            }
                        }
                        // ADIF: revoke stop after it is accepted
                        EmergencyStops.Remove(id);
                        if (stopAccepted != 3)
                        {
                            var revok = new ETCSVariables();
                            revok.Push(18, 8);
                            revok.Push(0, 10);
                            revok.Push(0, 32);
                            revok.Push(1, 1);
                            revok.Push(position?.LRBG ?? 16777215, 24);
                            revok.Push(id, 4);
                            SendMessage(revok, true);
                        }
                    }
                    break;
                case 157: // SoM position report
                    int status = (int)msg.Access(74, 2);
                    if (status == 1)
                    {
                    }
                    else if (TrainPosition == null || status == 2)
                    {
                        // Unknown position, accept train nevertheless
                        bool accept = true;
                        var accepted = new ETCSVariables();
                        accepted.Push(accept ? 41 : 40, 8);
                        accepted.Push(0, 10);
                        accepted.Push(0, 32);
                        accepted.Push(1, 1);
                        accepted.Push(16777215, 24);
                        SendMessage(accepted, true);
                    }
                    else
                    {
                        // Validate position report
                        var ackPosition = new ETCSVariables();
                        ackPosition.Push(43, 8);
                        ackPosition.Push(0, 10);
                        ackPosition.Push(0, 32);
                        ackPosition.Push(1, 1);
                        ackPosition.Push(position?.LRBG ?? 16777215, 24);
                        SendMessage(ackPosition, true);
                    }
                    break;
                case 132: // MA request
                    int reason = (int)msg.Access(74, 5);
                    bool fromStart = (reason & 1) != 0;
                    bool tafL23 = (reason & 16) != 0;
                    bool canGiveMA = false;
                    if (TrainPosition != null && (CurrentMode == Mode.SR || CurrentMode == Mode.OS || CurrentMode == Mode.FS || fromStart || tafL23))
                    {
                        var info = ProvideRouteInfo();
                        canGiveMA = TrySendInfo(info, tafL23);
                    }
                    // If driver pressed Start but cannot give MA: allow SR
                    if (!canGiveMA && fromStart && (CurrentMode == Mode.SR || CurrentMode == Mode.SB || CurrentMode == Mode.PT))
                    {
                        var sr = new ETCSVariables();
                        sr.Push(2, 8);
                        sr.Push(0, 10);
                        sr.Push(0, 32);
                        sr.Push(1, 1);
                        sr.Push(position?.LRBG ?? 16777215, 24);
                        sr.Push(1, 2);
                        sr.Push(32767, 15);
                        SendMessage(sr, true);
                    }
                    break;
                case 136: // Position report
                case 149: // TAF granted
                    if (TrainPosition != null)
                    {
                        if (CurrentMode == Mode.SR || CurrentMode == Mode.OS || (CurrentMode == Mode.FS && CurrentLevel == Level.L1) || (((CurrentMode == Mode.UN && CurrentLevel == Level.L0) || (CurrentMode == Mode.SN && CurrentLevel == Level.NTC)) && (L2TransitionOrder || MABeforeTransition)))
                        {
                            RouteInfo info = ProvideRouteInfo();
                            bool ma = TrySendInfo(info, nid_message == 149);
                            if (CurrentMode == Mode.OS && position.Value.MaxSafe >= info.NextSignal - 50) break;
                            // Send position report parameters
                            var rqpar = new ETCSVariables();
                            rqpar.Push(58, 8);
                            rqpar.Push(2, 2);
                            rqpar.Push(0, 13);
                            rqpar.Push(1, 2);
                            rqpar.Push(15, 8); // Every 15s
                            rqpar.Push(500, 15); // Every 500m
                            rqpar.Push(1, 3); // Every LRBG
                            if ((!ma && CurrentMode != Mode.FS) || CurrentMode == Mode.OS)
                            {
                                // Also send when reaching the 50m confidence area: transition to FS
                                rqpar.Push(1, 5);
                                rqpar.Push((int)info.NextSignal - 45, 15);
                                rqpar.Push(1, 1);
                            }
                            else
                            {
                                rqpar.Push(0, 5);
                            }
                            rqpar.Replace(10, rqpar.Length, 13);
                            OptionalPackets.Add(rqpar);
                        }
                    }
                    break;
                case 130: // Request SH
                    // Always allow transition to SH
                    var sh = new ETCSVariables();
                    sh.Push(28, 8);
                    sh.Push(0, 10);
                    sh.Push(0, 32);
                    sh.Push(1, 1);
                    sh.Push(position?.LRBG ?? 16777215, 24);
                    sh.Push(t_train, 32);
                    SendMessage(sh, true);
                    break;
                case 150: // End of mission
                    TerminateSession();
                    break;
                case 146: // Message ack
                    uint t_trainack = (uint)msg.Access(74, 32);
                    PendingAck.Remove(t_trainack);
                    break;
            }
            if (OptionalPackets.Count > 0)
            {
                var message = new ETCSVariables();
                message.Push(24, 8);
                message.Push(0, 10);
                message.Push(0, 32);
                message.Push(1, 1);
                message.Push(position?.LRBG ?? 16777215, 24);
                SendMessage(message, true);
            }
        }
        void RequestTAF(RouteInfo info)
        {
            int start = (int)info.NextSignal - 500;
            int length = (int)(info.NextSignal - start - 50);
            if (length < 5) return;
            float shift = 0;
            if (start < 0)
            {
                shift = -start;
                start = 0;
            }
            var taf = new ETCSVariables();
            taf.Push(34, 8);
            taf.Push(0, 10);
            taf.Push(0, 32);
            taf.Push(1, 1);
            taf.Push(position?.LRBG ?? 16777215, 24);
            taf.Push(1, 2);
            if (position.Value.BackFacing)
                taf.Push((int)shift, 16);
            else
                taf.Push((((int)shift)^65535)+1, 16);
            taf.Push(position.Value.BackFacing ? 0 : 1, 2);
            taf.Push(start, 15);
            taf.Push(length, 15);
            SendMessage(taf, true);
        }
        void TerminateSession()
        {
            var close = new ETCSVariables();
            close.Push(24, 8);
            close.Push(0, 10);
            close.Push(0, 32);
            close.Push(0, 1);
            close.Push(position?.LRBG ?? 16777215, 24);
            close.Push(42, 8);
            close.Push(2, 2);
            close.Push(113, 13);
            close.Push(0, 1);
            close.Push(NID_C, 10);
            close.Push(16383, 14);
            close.Push(0, 64);
            close.Push(1, 1);
            SendMessage(close, false);
        }
        DateTime LastUpdate;
        bool IsLevel2(SignalHead signal)
        {
            return signal != null && (signal.this_sig_lvar(200) & 2) != 0;
        }
        public void Update()
        {
            if ((DateTime.UtcNow - LastUpdate).TotalMilliseconds < 200) return;
            LastUpdate = DateTime.UtcNow;
            if (Connection == null || ReferenceTimeStamp == null) return;
            uint stamp = (uint)(DateTime.UtcNow.Subtract(LocalReferenceTime).TotalMilliseconds / 10) + (uint)ReferenceTimeStamp.Value;
            if (LastTimeStamp - stamp < 20) LastTimeStamp++;
            else LastTimeStamp = stamp;
            // Resend messages not yet acknowledged
            foreach (var key in PendingAck.Keys.ToList())
            {
                var v = PendingAck[key];
                PendingAck.Remove(key);
                if (DateTime.UtcNow.Subtract(v.Item2).TotalSeconds > 10)
                {
                    v.Item1.Replace(18, LastTimeStamp, 32);
                    Connection.Send(v.Item1);
                    v.Item2 = DateTime.UtcNow;
                    v.Item3++;
                    PendingAck[LastTimeStamp] = v;
                    LastTimeStamp++;
                }
            }
            if (LRBGPosition != null && TrainPosition != null && (SentInfo == null || CurrentLevel != Level.L2))
            {
                bool level2 = false;
                int signals = 0;
                for (int iindex=0; iindex<TrainRoute.Count; iindex++)
                {
                    var element = TrainRoute[iindex];
                    var tc = Signals.TrackCircuitList[element.TCSectionIndex];
                    int dir = element.Direction;
                    if (tc.EndSignals[dir] != null)
                    {
                        if (++signals > 4) break;
                        if (IsLevel2(tc.EndSignals[dir].SignalHeads[0]))
                        {
                            level2 = true;
                            break;
                        }
                    }
                }
                if (!level2)
                {
                    if (SpeedMpS > MpS.FromKpH(15) && CurrentLevel == Level.L2) EmergencyStop();
                    TerminateSession();
                }
            }
            RouteInfo? routeInfo = null;
            if (TrainPosition != null)
            {
                routeInfo = ProvideRouteInfo();
            }
            else
            {
                NextSignalCleared.signal = -1;
            }
            if (position != null && SentInfo != null)
            {
                int changed = -1;
                for (int i=0; i<SentInfo.Value.Signals.Count; i++)
                {
                    // ADIF: send emergency stop when a signal is closed
                    var sig = SentInfo.Value.Signals[i];
                    Aspecto nuevo = ConvertirAspecto(sig.Item1.TextSignalAspect);
                    if (nuevo != sig.Item2 && nuevo == Aspecto.Parada && (CurrentMode == Mode.FS || CurrentMode == Mode.OS))
                    {
                        var closed = sig.Item1.mainSignal;
                        float dist = TrainRoute.GetDistanceAlongRoute(LRBGPosition.RouteListIndex, Signals.TrackCircuitList[LRBGPosition.TCSectionIndex].Length - LRBGPosition.TCOffset, TrainRoute.GetRouteIndex(closed.TCReference, 0), closed.TCOffset, true, Signals) - 10;
                        if (dist < 0)
                        {
                            changed = i;
                            break;
                        }
                        var stop = new ETCSVariables();
                        EmergencyStops[EmergencyStopCount] = sig.Item1;
                        stop.Push(15, 8);
                        stop.Push(0, 10);
                        stop.Push(0, 32);
                        stop.Push(1, 1);
                        stop.Push(position?.LRBG ?? 16777215, 24);
                        stop.Push(EmergencyStopCount, 4);
                        stop.Push(1, 2);
                        if (dist < 0)
                        {
                            dist *= -1;
                            if (position.Value.BackFacing)
                                stop.Push((int)dist, 16);
                            else
                                stop.Push((((int)dist)^65535)+1, 16);
                            dist = 0;
                        }
                        else stop.Push(0, 16);
                        stop.Push(position.Value.BackFacing ? 0 : 1, 2);
                        stop.Push((int)dist, 15);
                        SendMessage(stop, true);
                        changed = i;
                        EmergencyStopCount = (EmergencyStopCount + 1) % 16;
                        break;
                    }
                }
                if (changed >= 0)
                {
                    var info = SentInfo.Value;
                    var item = info.Signals[changed];
                    item.Item2 = ConvertirAspecto(item.Item1.TextSignalAspect);
                    info.Signals[changed] = item;
                    SentInfo = info;
                }
            }
            if ((CurrentMode == Mode.SR || CurrentMode == Mode.UN || CurrentMode == Mode.SN) && EmergencyStops.Count != 0)
            {
                EmergencyStops.Clear();
            }
            if (routeInfo != null)
            {
                var newLX = new Dictionary<int,bool>();
                float offset = LRBGPosition.TCOffset;
                float totalLength = 0;
                for (int iindex=0; iindex<routeInfo.Value.ClearedRoute.Count; iindex++)
                {
                    var element = routeInfo.Value.ClearedRoute[iindex];
                    var tc = Signals.TrackCircuitList[element.TCSectionIndex];
                    int dir = element.Direction;
                    var lxs = tc.CircuitItems.TrackCircuitSignals[dir][Signals.SignalFunctions["OLPN_T"]].TrackCircuitItem;
                    foreach (var lx in lxs)
                    {
                        int id = (lx.SignalRef.thisRef&127)|128;
                        bool is_protected = lx.SignalRef.SignalHeads[0].state == MstsSignalAspect.APPROACH_2 || lx.SignalRef.SignalHeads[0].state == MstsSignalAspect.CLEAR_2;
                        float dist = lx.SignalLocation - offset + totalLength;
                        if (dist < 50) continue;
                        if (dist < position.Value.Distance) continue;
                        if (!LevelCrossings.ContainsKey(id) || LevelCrossings[id] != is_protected)
                        {
                            var pack = new ETCSVariables();
                            pack.Push(88, 8);
                            pack.Push(position.Value.BackFacing ? 0 : 1, 2);
                            pack.Push(0, 13);
                            pack.Push(1, 2);
                            pack.Push(id, 8);
                            pack.Push((int)dist - 50, 15);
                            pack.Push(30, 15);
                            if (is_protected) pack.Push(0, 1);
                            else
                            {
                                pack.Push(1, 1);
                                pack.Push(2, 7);
                                pack.Push(0, 1);
                            }
                            pack.Replace(10, pack.Length, 13);
                            OptionalPackets.Add(pack);
                        }
                        newLX[id] = is_protected;
                    }
                    totalLength += tc.Length - offset;
                    offset = 0;
                }
                LevelCrossings = newLX;
            }
            if (CurrentLevel == Level.L2 && (CurrentMode == Mode.FS || CurrentMode == Mode.OS) && EmergencyStops.Count == 0 && routeInfo != null)
            {
                // Extend MA if possible
                var info = routeInfo.Value;
                if (info.EndSignal != null)
                {
                    if (SentInfo == null || SentInfo.Value.EndSignal == null) TrySendInfo(info, false);
                    else if (info.EndSignal != SentInfo.Value.EndSignal)
                    {
                        int index1 = TrainRoute.GetRouteIndex(SentInfo.Value.EndSignal.mainSignal.TCReference, 0);
                        int index2 = TrainRoute.GetRouteIndex(info.EndSignal.mainSignal.TCReference, 0);
                        if (index1 < index2) TrySendInfo(info, false);
                    }
                }
            }
            // Reset T_NVCONTACT
            if (Established && (((CurrentMode == Mode.FS || CurrentMode == Mode.OS) && DateTime.UtcNow - Connection.LastSentMessage > TimeSpan.FromSeconds(10)) || OptionalPackets.Count > 0))
            {
                var empty = new ETCSVariables();
                empty.Push(24, 8);
                empty.Push(0, 10);
                empty.Push(0, 32);
                empty.Push(0, 1);
                empty.Push(position?.LRBG ?? 16777215, 24);
                SendMessage(empty, OptionalPackets.Count > 0);
            }
            if (Established && DateTime.UtcNow - LocalReferenceTime > TimeSpan.FromMinutes(5))
            {
                Established = false;
            }
        }
        void EmergencyStop()
        {
            var stop = new ETCSVariables();
            stop.Push(16, 8);
            stop.Push(0, 10);
            stop.Push(0, 32);
            stop.Push(0, 1);
            stop.Push(position?.LRBG ?? 16777215, 24);
            stop.Push(1, 4);
            SendMessage(stop, true);
        }
        int GetCoordinateSystem(int lrbg1, int lrbg2)
        {
            var sig1 = Signals.SignalObjects[lrbg1&16383];
            var sig2 = Signals.SignalObjects[lrbg2&16383];
            var tc1 = Signals.TrackCircuitList[sig1.TCReference];
            var tc2 = Signals.TrackCircuitList[sig2.TCReference];
            return 1;
        }
        public struct LinkingElement
        {
            public int SignalId;
            public double DistanceM;
            public bool Reverse;
            public LinkingElement(int id, double dist, bool rev)
            {
                SignalId = id;
                DistanceM = dist;
                Reverse = rev;
            }
        }
        public struct RouteInfo
        {
            public float MAEnd;
            public float NextSignal;
            public List<(SignalHead, Aspecto, float)> Signals;
            public Train.TCSubpathRoute ClearedRoute;
            public SignalHead EndSignal;
            public int? OSProfileStart;
            public int? OSProfileEnd;
            public float FirstL2Signal;
        }
        public enum Aspecto
        {
            Parada,
            ParadaLZB,
            ParadaSelectiva,
            ParadaSelectivaDestellos,
            RebaseAutorizado,
            RebaseAutorizadoCortaDistancia,
            RebaseAutorizadoDestellos,
            MovimientoAutorizado,
            ParadaDiferida,
            AnuncioParadaInmediata,
            AnuncioParada,
            AnuncioPrecaucion,
            PreanuncioParada,
            ViaLibreCondicional,
            ViaLibre,
            IndicadoraDesviada,
            IndicadoraDirecta,
        }
        Dictionary<string, Aspecto> textoAAspecto;
        Aspecto ConvertirAspecto(string text)
        {
            if (textoAAspecto.ContainsKey(text))
            {
                return textoAAspecto[text];
            }
            return Aspecto.Parada;
        }
        void BuildTrainRoute()
        {
            if (position == null)
            {
                TrainRoute.Clear();
                LRBGPosition = null;
                TrainPosition = null;
                return;
            }
            var lrbg = Signals.SignalObjects[position.Value.LRBG&16383];
            var tc = Signals.TrackCircuitList[lrbg.TCReference];
            bool reverse = position.Value.BackFacing;
            int dir = reverse ? (1-lrbg.TCDirection) : lrbg.TCDirection;
            float offset = reverse ? tc.Length - lrbg.TCOffset : lrbg.TCOffset;
            int index = TrainRoute.GetRouteIndex(lrbg.TCReference, 0);
            if (index >= 0) TrainRoute.RemoveRange(0, index);
            else TrainRoute.Clear();
            LRBGPosition = new Train.TCPosition()
            {
                TCSectionIndex = lrbg.TCReference,
                TCDirection = dir,
                TCOffset = offset,
                RouteListIndex = 0
            };
            if (TrainRoute.Count == 0) TrainRoute.Add(new Train.TCRouteElement(tc, dir, Signals, -2));
            // If route ends before train position, extend
            float totalLength = TrainRoute.Select(x => Signals.TrackCircuitList[x.TCSectionIndex].Length).Sum() - offset;
            if (totalLength <= position.Value.Distance)
            {
                var lastElement = TrainRoute[TrainRoute.Count-1];
                var tempSections = Signals.ScanRoute(null, lastElement.TCSectionIndex, 0, lastElement.Direction, true,
                                            position.Value.Distance - totalLength + Signals.TrackCircuitList[lastElement.TCSectionIndex].Length,
                                            true, false, false, false, true, false, false, false, false, false);
                tempSections.RemoveAt(0);
                int prevSection = lastElement.TCSectionIndex;
                foreach (int sectionIndex in tempSections)
                {
                    int sectionDirection = sectionIndex > 0 ? 0 : 1;
                    var element = new Train.TCRouteElement(Signals.TrackCircuitList[Math.Abs(sectionIndex)],
                            sectionDirection, Signals, prevSection);
                    TrainRoute.Add(element);
                    prevSection = Math.Abs(sectionIndex);
                }
            }

            TrainPosition = null;
            // Find train position
            totalLength = -LRBGPosition.TCOffset;
            if (position.Value.Distance < totalLength) return;
            for (int i=0; i<TrainRoute.Count && TrainPosition == null; i++)
            {
                var element = TrainRoute[i];
                tc = Signals.TrackCircuitList[element.TCSectionIndex];
                totalLength += tc.Length;
                if (position.Value.Distance < totalLength)
                {
                    TrainPosition = new Train.TCPosition()
                    {
                        TCSectionIndex = element.TCSectionIndex,
                        TCDirection = element.Direction,
                        TCOffset = position.Value.Distance - (totalLength - tc.Length),
                        RouteListIndex = i
                    };
                }
            }

            if (TrainPosition == null) return;

            // Remove route ahead of train
            TrainRoute.RemoveRange(TrainPosition.RouteListIndex, TrainRoute.Count - TrainPosition.RouteListIndex);

            // Extend route from the train
            {
                var tempSections = Signals.ScanRoute(null, TrainPosition.TCSectionIndex, TrainPosition.TCOffset, TrainPosition.TCDirection, true, 32000, true, false, false, false, true, false, false, false, false, false);
                int prevSection = TrainRoute.Count > 0 ? TrainRoute[TrainRoute.Count-1].TCSectionIndex : -2;
                foreach (int sectionIndex in tempSections)
                {
                    int sectionDirection = sectionIndex > 0 ? 0 : 1;
                    var element = new Train.TCRouteElement(Signals.TrackCircuitList[Math.Abs(sectionIndex)],
                            sectionDirection, Signals, prevSection);
                    TrainRoute.Add(element);
                    prevSection = Math.Abs(sectionIndex);
                }
            }
        }
        RouteInfo ProvideRouteInfo()
        {
            RouteInfo info = new RouteInfo()
            {
                NextSignal = -1,
                EndSignal = null,
                ClearedRoute = new Train.TCSubpathRoute(),
                Signals = new List<(SignalHead, Aspecto, float)>(),
                OSProfileStart = null,
                OSProfileEnd = null,
            };
            if (CurrentMode == Mode.OS) info.OSProfileStart = -1;
            float offset = LRBGPosition.TCOffset;
            float trainPosition = TrainRoute.GetDistanceAlongRoute(0, Signals.TrackCircuitList[LRBGPosition.TCSectionIndex].Length - LRBGPosition.TCOffset, TrainPosition.RouteListIndex, TrainPosition.TCOffset, true, Signals);
            float totalLength = 0;
            bool findPlatform = false;
            int? id1 = null;
            int? id2 = null;
            if (SentInfo != null && SentInfo.Value.OSProfileStart != null)
            {
                id1 = TrainRoute.GetRouteIndex(SentInfo.Value.OSProfileStart.Value, 0);
                if (SentInfo.Value.OSProfileEnd != null) id2 = TrainRoute.GetRouteIndex(SentInfo.Value.OSProfileEnd.Value, 0);
            }
            for (int iindex=0; iindex<TrainRoute.Count; iindex++)
            {
                var element = TrainRoute[iindex];
                var tc = Signals.TrackCircuitList[element.TCSectionIndex];
                int dir = element.Direction;
                totalLength += tc.Length - offset;
                offset = 0;
                info.ClearedRoute.Add(element);
                if (findPlatform && tc.PlatformIndex.Count > 0)
                {
                    info.OSProfileStart = tc.Index;
                    findPlatform = false;
                }
                if (tc.EndSignals[dir] != null)
                {
                    var signal = tc.EndSignals[dir];
                    bool retroceso = signal.this_sig_hasnormalsubtype(Signals.ORTSNormalsubtypes.IndexOf("RETROCESO")) != 0;
                    bool pantalla = signal.this_sig_hasnormalsubtype(Signals.ORTSNormalsubtypes.IndexOf("PANTALLA_ERTMS")) != 0;
                    Aspecto asp = ConvertirAspecto(signal.SignalHeads[0].TextSignalAspect);
                    bool level2 = IsLevel2(signal.SignalHeads[0]);
                    if (iindex >= TrainPosition.RouteListIndex)
                    {
                        if (info.NextSignal < 0 && !retroceso)
                        {
                            if (level2)
                            {
                                info.NextSignal = totalLength;
                                if (asp != Aspecto.Parada) NextSignalCleared = (signal.thisRef, asp, DateTime.UtcNow);
                                if (AllowOSbeforeFS && info.OSProfileStart == null && totalLength - trainPosition > 50)
                                {
                                    info.OSProfileStart = -1;
                                    if (asp != Aspecto.RebaseAutorizadoDestellos) id2 = signal.TCReference;
                                }
                            }
                            else info.NextSignal = float.MaxValue;
                        }
                        info.Signals.Add((signal.SignalHeads[0], asp, totalLength));
                        info.EndSignal = signal.SignalHeads[0];
                        if (id1 != null && id1 <= iindex && info.OSProfileStart == null)
                        {
                            id1 = null;
                            info.OSProfileStart = SentInfo.Value.OSProfileStart;
                        }
                        if (pantalla && id2 == iindex && info.OSProfileStart != null && info.OSProfileEnd == null)
                        {
                            id1 = id2 = null;
                            if (asp != Aspecto.RebaseAutorizadoDestellos) info.OSProfileEnd = SentInfo.Value.OSProfileEnd;
                        }
                        if (!retroceso && !pantalla) id1 = id2 = null;
                        bool rebase = info.OSProfileStart.HasValue && !info.OSProfileEnd.HasValue;
                        if (pantalla && (rebase || id1 != null))
                        {
                            if (asp != Aspecto.Parada || info.OSProfileStart.HasValue) continue;
                        }
                        if (!retroceso && !pantalla && rebase)
                        {
                            findPlatform = false;
                            if (totalLength - trainPosition < 50)
                            {
                                if (asp != Aspecto.RebaseAutorizadoDestellos) info.OSProfileEnd = signal.TCReference;
                            }
                            else
                            {
                                info.OSProfileEnd = signal.TCReference;
                                break;
                            }
                        }

                        if (NextSignalCleared.signal == signal.thisRef && (DateTime.UtcNow-NextSignalCleared.time).Seconds < 10 &&
                            asp == Aspecto.Parada && NextSignalCleared.aspecto != Aspecto.Parada && info.NextSignal - position.Value.MaxSafe < 50)
                            asp = NextSignalCleared.aspecto;
                        if (asp == Aspecto.RebaseAutorizadoDestellos)
                        {
                            if (!info.OSProfileStart.HasValue)
                            {
                                info.OSProfileStart = signal.TCNextTC;
                                findPlatform = true;
                            }
                        }
                        if (asp == Aspecto.Parada || asp == Aspecto.RebaseAutorizado || asp == Aspecto.RebaseAutorizadoCortaDistancia) break;
                        if (asp == Aspecto.ParadaLZB || asp == Aspecto.ParadaSelectiva)
                        {
                            if ((signal.SignalHeads[0].this_sig_lvar(200) & 10) == 0) break;
                        }
                        if (asp == Aspecto.ParadaSelectivaDestellos)
                        {
                            if ((signal.this_sig_lvar(200) & 11) == 0) break;
                        }
                        if (!retroceso && level2 && CurrentLevel == Level.L2 && CurrentMode == Mode.FS && ERTMSRoutes) signal.SignalHeads[0].usedCsSignalScript?.HandleSignalMessage(-1, "ERTMS_ROUTE");
                    }
                    else if (level2 && !retroceso && position.Value.MinSafe - totalLength < 50 && NextSignalCleared.signal == signal.thisRef && (DateTime.UtcNow-NextSignalCleared.time).Seconds < 10 && NextSignalCleared.aspecto != Aspecto.Parada)
                    {
                        if (NextSignalCleared.aspecto != Aspecto.RebaseAutorizadoDestellos && asp != Aspecto.RebaseAutorizadoDestellos) info.OSProfileStart = null;
                        else if (!info.OSProfileStart.HasValue)
                        {
                            info.OSProfileStart = signal.TCNextTC;
                            if (!retroceso && !pantalla)
                            {
                                id2 = null;
                                findPlatform = true;
                            }
                            id1 = null;
                        }
                        info.NextSignal = totalLength;
                    }
                }
                if (totalLength > 32000) break;
            }
            info.MAEnd = Math.Min(totalLength, 32700);
            if (info.NextSignal < 0) info.NextSignal = float.MaxValue;
            return info;
        }
        void SendLinking()
        {
            foreach (var pack in OptionalPackets)
            {
                if (pack.Access(0, 8) == 5) return;
            }
            float offset = LRBGPosition.TCOffset;
            float totalLength = 0;
            List<LinkingElement> balises = new List<LinkingElement>();
            foreach (var element in TrainRoute)
            {
                if (balises.Count > 5) break;
                var tc = Signals.TrackCircuitList[element.TCSectionIndex];
                float endLength = totalLength + tc.Length - offset;
                balises.AddRange(tc.CircuitItems.TrackCircuitSignals[element.Direction][Signals.SignalFunctions["ETCS"]].TrackCircuitItem
                .Where(
                    s => s.SignalLocation > offset && s.SignalRef.SignalHeads[0].TextSignalAspect.Substring(9, 3) == "000"
                ).Select(
                    s => new LinkingElement(s.SignalRef.thisRef, s.SignalLocation + totalLength - offset, false)));
                /*balises.AddRange(tc.CircuitItems.TrackCircuitSignals[1-element.Direction][Signals.SignalFunctions["ETCS"]].TrackCircuitItem
                .Where(
                    s => tc.Length - s.SignalLocation > offset && s.SignalRef.SignalHeads[0].TextSignalAspect.Substring(9, 3) == "000"
                ).Select(
                    s => new LinkingElement(s.SignalRef.thisRef, tc.Length - s.SignalLocation + totalLength - offset, true)));*/
                offset = 0;
                totalLength = endLength;
            }
            balises.Sort((x,y) => x.DistanceM.CompareTo(y.DistanceM));
            if (balises.Count > 0)
            {
                var msg = new ETCSVariables();
                msg.Push(5, 8);
                msg.Push(position.Value.BackFacing ? 0 : 1, 2);
                msg.Push(0, 13);
                msg.Push(1, 2);
                double last = 0;
                for (int i = 0; i < balises.Count; i++)
                {
                    msg.Push((int)(balises[i].DistanceM - last), 15);
                    msg.Push(0, 1);
                    msg.Push(balises[i].SignalId, 14);
                    msg.Push(balises[i].Reverse ? 0 : 1, 1);
                    msg.Push(2, 2);
                    msg.Push(3, 6);
                    last = balises[i].DistanceM;
                    if (i == 0) msg.Push(balises.Count - 1, 5);
                }
                msg.Replace(10, msg.Length, 13);
                OptionalPackets.Add(msg);
            }
        }
        bool TrySendInfo(RouteInfo info, bool taf)
        {
            if (EmergencyStops.Count != 0 || !TrainDataAcknowledged) return false;
            bool reverse = position.Value.BackFacing;
            // ADIF: transition to L2 FS only allowed when distance to main signal < 50m
            // If already in FS, route is assumed to be cleared up to next signal, so MA is always extended if possible
            bool confidenceArea = info.NextSignal - position.Value.MaxSafe < (taf ? 500 : 50) && position.Value.MinSafe - info.NextSignal < 50;
            float levelTransitionLocation = float.MaxValue;
            SignalHead l2TransitionSignal = null;
            if (CurrentMode == Mode.SR && info.Signals.Count > 0 && info.Signals[info.Signals.Count - 1].Item3 - position.Value.MaxSafe < 50) return false;
            if (CurrentLevel == Level.L2)
            {
                if (confidenceArea) {}
                else if (CurrentMode == Mode.FS)
                {
                    bool extend = false;
                    if (SentInfo != null && info.Signals.Count > 0)
                    {
                        foreach (var t in SentInfo.Value.Signals)
                        {
                            if (t.Item1 == info.Signals[0].Item1)
                            {
                                extend = true;
                                break;
                            }
                        }
                    }
                    if (!extend) return false;
                }
                else if ((AllowOSbeforeFS || CurrentMode == Mode.OS) && info.NextSignal < 1e6)
                {
                    if (AllowTAF) RequestTAF(info);
                }
                else
                {
                    if ((CurrentMode == Mode.SR || CurrentMode == Mode.OS || CurrentMode == Mode.PT || CurrentMode == Mode.SB) && AllowTAF) RequestTAF(info);
                    return false;
                }
            }
            else
            {
                foreach (var signal in info.Signals)
                {
                    if (IsLevel2(signal.Item1))
                    {
                        levelTransitionLocation = signal.Item3;
                        l2TransitionSignal = signal.Item1;
                        break;
                    }
                }
                if (confidenceArea) {}
                else if (MABeforeTransition)
                {
                    if (levelTransitionLocation - position.Value.Distance > Math.Max(SpeedMpS * 15, 1500)) return false;
                }
                else return false;
            }
            SentInfo = info;

            float shift = position.Value.Distance < 0 ? LRBGPosition.TCOffset : 0;
            info.MAEnd += shift;

            float CurrentSpeedLimit;
            var SpeedPosts = new List<(double, ObjectSpeedInfo)>();
            var backSpeedPosts = new List<(double, ObjectSpeedInfo)>();
            var Gradient = new List<(double, double)>();
            var TrackConditions = new List<(double, double, int)>();
            {  
                double currentGradient = float.MaxValue;
                double currentAltitude = Signals.SignalObjects[position.Value.LRBG&16383].tdbtraveller.WorldLocation.Location.Y;
                double prevGradDist = 0;
                double currentGradDist = 0;
                float offset = LRBGPosition.TCOffset - shift;
                float totalLength = 0;
                var tunnels = new List<(double, double)>();
                var infoSignals = new List<(double, SignalObject)>();
                List<string> ignoreSpeedPosts = new List<string>{"placa_2dig","placa_3dig","placa_4dig","lav_rasante1","lav_rasante2","lav_rasante3","lav_rasante4"};
                for (int iindex=0; iindex<info.ClearedRoute.Count; iindex++)
                {
                    var element = info.ClearedRoute[iindex];
                    var tc = Signals.TrackCircuitList[element.TCSectionIndex];
                    int dir = element.Direction;
                    
                    foreach (var s in tc.CircuitItems.TrackCircuitSpeedPosts[dir].TrackCircuitItem)
                    {
                        if (s.SignalLocation > offset && !ignoreSpeedPosts.Contains(s.SignalRef.SignalHeads[0].SignalTypeName.ToLowerInvariant()))
                        {
                            var speed = s.SignalRef.this_lim_speed(SignalFunction.SPEED);
                            if (!speed.speed_isWarning && speed.speed_pass > 0 && speed.speed_pass < 900)
                                SpeedPosts.Add(((double)s.SignalLocation + totalLength, speed));
                        }
                    }
                    foreach (var s in tc.CircuitItems.TrackCircuitSpeedPosts[1-dir].TrackCircuitItem)
                    {
                        if (tc.Length - s.SignalLocation > offset && !ignoreSpeedPosts.Contains(s.SignalRef.SignalHeads[0].SignalTypeName.ToLowerInvariant()))
                        {
                            var speed = s.SignalRef.this_lim_speed(SignalFunction.SPEED);
                            if (!speed.speed_isWarning && speed.speed_pass > 0 && speed.speed_pass < 900)
                                backSpeedPosts.Add(((double)tc.Length - s.SignalLocation + totalLength, speed));
                        }
                    }

                    if (tc.TunnelInfo != null)
                    {
                        foreach (var thisTunnel in tc.TunnelInfo)
                        {
                            var sectionDistanceToTrainM = totalLength - offset;
                            var tunnelStartOffset = thisTunnel[dir].TunnelStart;
                            if (tunnelStartOffset > offset)
                            {
                                tunnels.Add((tunnelStartOffset + totalLength - offset, thisTunnel[dir].TotalLength));
                            }
                            else if (tunnels.Count == 0 && (thisTunnel[dir].TunnelEnd < 0 || thisTunnel[dir].TunnelEnd > offset))
                            {
                                // Train is in tunnel, compute remaining length
                                var remainingLength = thisTunnel[dir].TotalLength - offset + (tunnelStartOffset < 0 ? -thisTunnel[dir].TCSStartOffset : tunnelStartOffset);
                                tunnels.Add((0, remainingLength));
                            }
                            else break;
                        }
                    }
                    infoSignals.AddRange(tc.CircuitItems.TrackCircuitSignals[dir][Signals.SignalFunctions["INFO"]].TrackCircuitItem.Where(s => s.SignalLocation >= offset).Select(s => ((double)s.SignalLocation + totalLength, s.SignalRef)).ToList());
                    {
                        if (tc.EndSignals[dir] != null)
                        {
                            float dist = totalLength+tc.Length-offset;
                            float alt = tc.EndSignals[dir].tdbtraveller.WorldLocation.Location.Y;
                            Gradient.Add((currentGradDist, (alt-currentAltitude)/(dist-currentGradDist)*1000));
                            currentGradDist = dist;
                            currentAltitude = alt;
                        }
                        /*double dist = totalLength-offset;
                        var thisNode = Signals.trackDB.TrackNodes[tc.OriginalIndex];
                        if (thisNode.TrVectorNode != null && thisNode.TrVectorNode.TrVectorSections != null)
                        {
                            foreach (TrVectorSection thisSection in (dir == 1 ? thisNode.TrVectorNode.TrVectorSections.Reverse() : thisNode.TrVectorNode.TrVectorSections))
                            {
                                double length = 0;
                                if (Signals.Simulator.TSectionDat.TrackSections.TryGetValue(thisSection.SectionIndex, out var TS))
                                {
                                    if (TS.SectionCurve != null)
                                    {
                                        length = Math.Abs(TS.SectionCurve.Angle) * Math.PI / 180 * TS.SectionCurve.Radius;
                                    }
                                    else
                                    {
                                        length = TS.SectionSize.Length;
                                    }
                                }
                                dist += length/2;
                                if (dist <= 0)
                                {
                                    dist += length/2;
                                    continue;
                                }
                                double alt = thisSection.Y;
                                //double grad = (alt - currentAltitude)/(dist-currentGradDist)*1000;
                                double grad = thisSection.AX*1000;
                                if (currentGradient < 1000 && currentGradDist - prevGradDist > 500 && Math.Abs(grad-currentGradient)>5)
                                {
                                    if (Gradient.Count > 0 && Math.Abs(Gradient[Gradient.Count-1].Item2-currentGradient) < 5)
                                    {
                                        var prev = Gradient[Gradient.Count-1];
                                        Gradient[Gradient.Count-1] = (prev.Item1, Math.Min(currentGradient, prev.Item2));
                                    }
                                    else Gradient.Add((prevGradDist, currentGradient));
                                    prevGradDist = currentGradDist;
                                    currentGradient = grad;
                                }
                                else if (grad < currentGradient)
                                {
                                    currentGradient = grad;
                                }
                                currentAltitude = alt;
                                currentGradDist = dist;
                                dist += length/2;
                            }
                        }*/
                    }
                    totalLength += tc.Length - offset;
                    offset = 0;
                }

                if (Gradient.Count == 0) Gradient.Add((0, 0));
                if (Gradient.Count > 32)
                {
                    double min = float.MaxValue;
                    for (int i=31; i<Gradient.Count; i++)
                    {
                        min = Math.Min(min, Gradient[i].Item2);
                    }
                    Gradient.RemoveRange(32, Gradient.Count-32);
                    Gradient[31] = (Gradient[31].Item1, min);
                }

                CurrentSpeedLimit = 350/3.6f;
                foreach (var sp in SpeedPosts)
                {
                    CurrentSpeedLimit = (float)sp.Item2.speed_pass;
                    break;
                }
                foreach (var sp in backSpeedPosts)
                {
                    CurrentSpeedLimit = (float)sp.Item2.speed_pass;
                    break;
                }

                List<int> speedpostList = Signals.ScanRoute(null, LRBGPosition.TCSectionIndex, LRBGPosition.TCOffset - shift,
                                    LRBGPosition.TCDirection, false, -1, false, true, false, false, false, false, false, true, false, false);
                if (speedpostList.Count > 0)
                {
                    var thisSpeedpost = Signals.SignalObjects[speedpostList[0]];
                    var sp = thisSpeedpost.this_lim_speed(SignalFunction.SPEED);
                    if (!ignoreSpeedPosts.Contains(thisSpeedpost.SignalHeads[0].SignalTypeName) && sp.speed_pass > 0 && sp.speed_pass < 900 && !sp.speed_isWarning)
                        CurrentSpeedLimit = sp.speed_pass;
                }

                TrackConditions.AddRange(tunnels.Select(t => (t.Item1, t.Item2, 0)));
                TrackConditions.AddRange(tunnels.Select(t => (t.Item1, t.Item2, 5))); 
                // Neutral sections
                for (int i = 0; i < infoSignals.Count; i++)
                {
                    var element = infoSignals[i];
                    string name = element.Item2.SignalHeads[0].SignalTypeName;
                    string endname = "";
                    int type = 15;
                    if (name == "iniciozonaneutra")
                    {
                        type = 9;
                        endname = "finzonaneutra";
                    }
                    else if (name == "startneutro")
                    {
                        type = 9;
                        endname = "endneutro";
                    }
                    else if (name == "bajarpantografos")
                    {
                        type = 3;
                        endname = "subirpantografos";
                    }
                    else if (name == "inicioviaducto")
                    {
                        type = 0;
                        endname = "finviaducto";
                    }
                    else if (name == "iniciopuente")
                    {
                        type = 0;
                        endname = "finpuente";
                    }
                    else continue;
                    double length = 200;
                    for (int j = 0; i+j < infoSignals.Count; j++)
                    {
                        var element2 = infoSignals[i + j];
                        if (element2.Item2.SignalHeads[0].SignalTypeName == endname)
                        {
                            length = element2.Item1-element.Item1;
                            break;
                        }
                    }
                    if (type == 3 || type == 9) TrackConditions.Add((element.Item1, length, 0));
                    TrackConditions.Add((element.Item1, length, type));
                }
                TrackConditions.Sort((x, y) => x.Item1.CompareTo(y.Item1));
            }
            var msg = new ETCSVariables();
            if (shift == 0)
            {
                msg.Push(3, 8); // Movement authority msg
                msg.Push(0, 10);
                msg.Push(0, 32);
                msg.Push(1, 1);
                msg.Push(position.Value.LRBG, 24);
            }
            else
            {
                msg.Push(33, 8); // Movement authority msg with shifted reference
                msg.Push(0, 10);
                msg.Push(0, 32);
                msg.Push(1, 1);
                msg.Push(position.Value.LRBG, 24);
                msg.Push(1, 2);
                if (position.Value.BackFacing)
                    msg.Push((int)shift, 16);
                else
                    msg.Push((((int)shift)^65535)+1, 16);
            }

            msg.Push(15, 8); // MA
            msg.Push(reverse ? 0 : 1, 2);
            msg.Push(66, 13);
            msg.Push(1, 2);
            msg.Push(0, 7);
            msg.Push(1023, 10);
            msg.Push(0, 5);
            msg.Push((int)info.MAEnd, 15);
            msg.Push(0, 1);
            msg.Push(0, 1);
            msg.Push(1, 1);
            bool topera = (info.EndSignal.this_sig_lvar(201) & 256) != 0;
            msg.Push(topera ? 10 : 50, 15);
            if (topera) msg.Push(2, 7);
            else if (ConvertirAspecto(info.EndSignal.TextSignalAspect) != Aspecto.Parada) msg.Push(6, 7);
            else msg.Push(0, 7);
            msg.Push(0, 1);

            int start;
            if (info.OSProfileStart.HasValue)
            {
                int id1 = info.ClearedRoute.GetRouteIndex(info.OSProfileStart.Value, 0);
                float length = info.MAEnd;
                float profileStart = 0;
                int id2 = info.ClearedRoute.GetRouteIndex(info.OSProfileEnd ?? -1, Math.Max(id1, 0));
                if (id1 >= 0)
                {
                    profileStart = info.ClearedRoute.GetDistanceAlongRoute(0, Signals.TrackCircuitList[LRBGPosition.TCSectionIndex].Length - (LRBGPosition.TCOffset - shift), id1, -25, true, Signals);
                    if (id2 >= id1) length = info.ClearedRoute.GetDistanceAlongRoute(id1, Signals.TrackCircuitList[info.OSProfileStart.Value].Length + 25, id2, Signals.TrackCircuitList[info.OSProfileEnd.Value].Length, true, Signals);
                }
                else if (id2 >= 0)
                {
                    length = info.ClearedRoute.GetDistanceAlongRoute(0, Signals.TrackCircuitList[LRBGPosition.TCSectionIndex].Length - (LRBGPosition.TCOffset - shift), id2, Signals.TrackCircuitList[info.OSProfileEnd.Value].Length, true, Signals);
                }
                start = msg.Length;
                msg.Push(80, 8); // OS mode profile
                msg.Push(reverse ? 0 : 1, 2);
                msg.Push(0, 13);
                msg.Push(1, 2);
                msg.Push((int)profileStart, 15);
                msg.Push(0, 2);
                msg.Push(6, 7);
                msg.Push((int)length, 15);
                msg.Push(300, 15);
                msg.Push(1, 1);
                msg.Push(0, 5);
                msg.Replace(start + 10, msg.Length - start, 13);
            }

            start = msg.Length;
            msg.Push(21, 8); // Gradient profile
            msg.Push(reverse ? 0 : 1, 2);
            msg.Push(0, 13);
            msg.Push(1, 2);
            double last = 0;
            for (int i=0; i<Gradient.Count; i++) {
                var gr = Gradient[i];
                msg.Push((int)(gr.Item1 - last), 15);
                msg.Push(gr.Item2 > 0 ? 1 : 0, 1);
                msg.Push((int)Math.Abs(gr.Item2), 8);
                if (i==0) msg.Push(Gradient.Count-1, 5);
                last = gr.Item1;
            }
            msg.Replace(start + 10, msg.Length - start, 13);

            start = msg.Length;
            msg.Push(27, 8); // SSP
            msg.Push(reverse ? 0 : 1, 2);
            msg.Push(0, 13);
            msg.Push(1, 2);
            msg.Push(0, 15);
            msg.Push((int)(CurrentSpeedLimit * 3.6 / 5), 7);
            msg.Push(0, 1);
            msg.Push(0, 5);
            msg.Push(0, 5);
            last = 0;
            int total = 0;
            foreach (var sp in SpeedPosts)
            {
                if (sp.Item1 - last > 32700) break;
                msg.Push((int)(sp.Item1-last), 15);
                msg.Push((int)(sp.Item2.speed_pass * 3.6 / 5), 7);
                msg.Push(0, 1);
                msg.Push(0, 5);
                last = sp.Item1;
                total++;
            }
            msg.Replace(start + 53, total, 5);
            msg.Replace(start + 10, msg.Length - start, 13);

            if (TrackConditions.Count != 0)
            {
                start = msg.Length;
                msg.Push(68, 8); // Track conditions
                msg.Push(reverse ? 0 : 1, 2);
                msg.Push(0, 13);
                msg.Push(1, 2);
                msg.Push(0, 1);
                double dist = 0;
                for (int i=0; i<TrackConditions.Count; i++)
                {
                    var cond = TrackConditions[i];
                    msg.Push((int)(cond.Item1 - dist), 15);
                    msg.Push((int)cond.Item2, 15);
                    msg.Push(cond.Item3, 4);
                    dist = cond.Item1;
                    if (i == 0) msg.Push(TrackConditions.Count - 1, 5);
                }
                msg.Replace(start + 10, msg.Length - start, 13);
            }

            if (CurrentLevel != Level.L2)
            {
                float distanceToTrain = levelTransitionLocation - position.Value.Distance;
                bool announce = LevelTrAnnouncement != Level.L2 && L2TransitionAnnouncementWithMA && distanceToTrain < Math.Max(SpeedMpS * 15, 1500);
                bool order = L2TransitionOrder && distanceToTrain < 50;
                if (announce || order)
                {
                    float lack = Math.Max(SpeedMpS * 5, 300);
                    start = msg.Length;
                    msg.Push(41, 8);
                    msg.Push(reverse ? 0 : 1, 2);
                    msg.Push(0, 13);
                    msg.Push(1, 2);
                    msg.Push(order ? 32767 : (int)(levelTransitionLocation + shift + 50), 15);
                    msg.Push(3, 3);
                    msg.Push((int)lack, 15);
                    List<(int,int)> backupLevels = new List<(int,int)>();
                    if (l2TransitionSignal != null)
                    {
                        int r = l2TransitionSignal.this_sig_lvar(200);
                        if ((r & 1) != 0) backupLevels.Add((2, 0));
                        if ((r & 8) != 0) backupLevels.Add((1, 10));
                        if ((r & 4) != 0) backupLevels.Add((1, 0));
                    }
                    backupLevels.Add((0, 0));
                    msg.Push(backupLevels.Count, 5);
                    for (int i=0; i<backupLevels.Count; i++)
                    {
                        msg.Push(backupLevels[i].Item1, 3);
                        if (backupLevels[i].Item1 == 1) msg.Push(backupLevels[i].Item2, 8);
                        msg.Push((int)lack, 15);
                    }
                    msg.Replace(start + 10, msg.Length - start, 13);
                    LevelTrAnnouncement = Level.L2;
                }
            }

            SendMessage(msg, true);
            return true;
        }
        TrainPosition? ReadPositionReport(ETCSVariables pack)
        {
            var nid_packet = pack.Read(8);
            var length = pack.Read(13);
            if (nid_packet == 1) // Report based on two BG
            {
                int scale = (int)pack.Read(2);
                int lrbg = (int)pack.Read(24);
                int lrbg2 = (int)pack.Read(24);
                float dist = pack.ReadDistanceM(scale);
                int trainOrientation = (int)pack.Read(2);
                int lrbgSide = (int)pack.Read(2);
                float maxsafe = dist + pack.ReadDistanceM(scale);
                float minsafe = dist - pack.ReadDistanceM(scale);
                int q_length = (int)pack.Read(2);
                if (q_length == 1 || q_length == 2) pack.Read(15);
                SpeedMpS = pack.ReadSpeedMpS();
                int trainDirection = (int)pack.Read(2);
                CurrentMode = (Mode)pack.Read(4);
                CurrentLevel = (Level)pack.Read(3);
                if (CurrentLevel == Level.NTC)
                {
                    int nid_ntc = (int)pack.Read(8);
                }
                if (lrbg == 16777215 || lrbg2 == 16777215) return null;
                // We need to assign a coordinate system to get train orientation
                int coord = GetCoordinateSystem(lrbg, lrbg2);
                if (coord != -1) // We could determine the path from lrbg2 to lrbg
                {
                    if (coord == 0)
                    {
                        trainDirection = 1 - trainDirection;
                        trainOrientation = 1 - trainOrientation;
                        lrbgSide = 1 - lrbgSide;
                    }
                    if (CurrentMode != Mode.FS && CurrentMode != Mode.OS)
                    {
                        // Send train orientation w.r.t LRBG
                        var assignment = new ETCSVariables();
                        assignment.Push(45, 8);
                        assignment.Push(0, 10);
                        assignment.Push(0, 32);
                        assignment.Push(0, 1);
                        assignment.Push(lrbg, 24);
                        assignment.Push(coord, 1);
                        SendMessage(assignment, true);
                    }
                    else if (TrainPosition != null)
                    {
                        SendLinking();
                    }
                    if (lrbgSide != trainOrientation)
                    {
                        maxsafe = maxsafe - 2 * dist;
                        minsafe = minsafe - 2 * dist;
                        dist = -dist;
                    }
                    return new TrainPosition
                    {
                        LRBG = lrbg,
                        Distance = dist,
                        MaxSafe = maxsafe,
                        MinSafe = minsafe,
                        BackFacing = trainOrientation == 0
                    };
                }
            }
            else if (nid_packet == 0)
            {
                int scale = (int)pack.Read(2);
                int lrbg = (int)pack.Read(24);
                float dist = pack.ReadDistanceM(scale);
                int trainOrientation = (int)pack.Read(2);
                int lrbgSide = (int)pack.Read(2);
                float maxsafe = dist + pack.ReadDistanceM(scale);
                float minsafe = dist - pack.ReadDistanceM(scale);
                int q_length = (int)pack.Read(2);
                if (q_length == 1 || q_length == 2) pack.Read(15);
                SpeedMpS = pack.ReadSpeedMpS();
                int trainDirection = (int)pack.Read(2);
                CurrentMode = (Mode)pack.Read(4);
                CurrentLevel = (Level)pack.Read(3);
                if (CurrentLevel == Level.NTC)
                {
                    int nid_ntc = (int)pack.Read(8);
                }
                if (lrbg == 16777215) return null;
                //if (trainOrientation != trainDirection) speed = -speed;
                if (lrbgSide != trainOrientation)
                {
                    maxsafe = maxsafe - 2 * dist;
                    minsafe = minsafe - 2 * dist;
                    dist = -dist;
                }
                return new TrainPosition
                {
                    LRBG = lrbg,
                    Distance = dist,
                    MaxSafe = maxsafe,
                    MinSafe = minsafe,
                    BackFacing = trainOrientation == 0
                };
            }
            return null;
        }
        void SendMessage(ETCSVariables msg, bool ack)
        {
            int nid = (int)msg.Access(0, 8);
            if (EmergencyStops.Count != 0 && nid != 15 && nid != 16 && nid != 18) return;
            if (OptionalPackets.Count > 0 && (nid == 3 || nid == 24) && ack)
            {
                foreach (var pack in OptionalPackets)
                {
                    msg.Append(pack);
                }
                OptionalPackets.Clear();
            }
            msg.Replace(18, LastTimeStamp, 32);
            if (ack) PendingAck[LastTimeStamp] = (msg, DateTime.UtcNow, 0);
            Connection?.Send(msg);
            LastTimeStamp++;
        }
    }

    public static class SocketExtensions
    {
        public static Task<int> ReceiveAsync(this Socket socket, byte[] buffer, int offset, int size)
        {
            return Task.Factory.FromAsync(
                (cb, state) => socket.BeginReceive(buffer, offset, size, SocketFlags.None, cb, state),
                (ar) => socket.EndReceive(ar),
                null
            );
        }
    }
    public class RBC_connection
    {
        Socket s;
        RBC_session session;
        enum ConnectionState
        {
            IDLE,
            WFAU3,
            WFRESP,
            DATA
        }
        ConnectionState State;
        //Mutex mutex = new Mutex();
        public DateTime LastSentMessage;
        public static Dictionary<int, RBC_session> sessions = new Dictionary<int, RBC_session>();
        public bool Active;
        public RBC_connection(Socket s)
        {
            this.s = s;
            s.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.KeepAlive, true);
            s.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.NoDelay, true);
            /*s.SetSocketOption(SocketOptionLevel.Tcp, SocketOptionName.TcpKeepAliveTime, 12);
            s.SetSocketOption(SocketOptionLevel.Tcp, SocketOptionName.TcpKeepAliveInterval, 3);
            s.SetSocketOption(SocketOptionLevel.Tcp, SocketOptionName.TcpKeepAliveRetryCount, 3);*/
            s.ReceiveTimeout = 0;
            Active = true;
        }
        Task<byte[]> ReadTask = null;
        async Task<byte[]> ReadTCP()
        {
            byte[] sizeb = new byte[2];
            int read = 0;
            while (read < 2)
            {
                int res = await s.ReceiveAsync(sizeb, read, 2-read);
                if (res <= 0) return null;
                read += res;
            }
            byte[] data = new byte[2 + (sizeb[0]<<8) + sizeb[1]];
            data[0] = sizeb[0];
            data[1] = sizeb[1];
            while (read < data.Length)
            {
                int res = await s.ReceiveAsync(data, read, data.Length-read);
                if (res <= 0) return null;
                read += res;
            }
            if (data[7] == 1) await Task.Delay(5000);
            else if (data[7] == 3) await Task.Delay(500);
            Console.Write("Reading: ");
            for (int i = 0; i < data.Length; i++)
            {
                Console.Write(data[i]);
                Console.Write(" ");
            }
            Console.WriteLine();
            return data;
        }
        void HandleALE(byte[] data)
        {
            int num = data[7];
            int offset = num == 1 ? 19 : 10;
            byte[] sa = new byte[data.Length-offset];
            Buffer.BlockCopy(data, offset, sa, 0, sa.Length);
            HandleSaPDU(sa);
        }
        void HandleSaPDU(byte[] data)
        {
            int type = (data[0]>>1)&15;
            switch (type)
            {
                case 1:
                {
                    State = ConnectionState.WFAU3;
                    byte[] res = new byte[21];
                    res[0] = (byte)((1<<5)|(2<<1)|1);
                    int rbc=(RBC_session.NID_C<<14)|RBC_session.NID_RBC;
                    for (int i=0; i<3; i++)
                    {
                        res[i+1] = (byte)((rbc>>((2-i)*8))&255);
                    }
                    res[4] = 1;
                    Random rnd = new Random();
                    for (int i=0; i<4; i++)
                    {
                        res[5+i] = (byte)rnd.Next(0, 255);
                    }
                    SendALE(res);
                    break;
                }
                case 3:
                {
                    State = ConnectionState.DATA;
                    byte[] res = new byte[9];
                    res[0] = (byte)((9<<1)|1);
                    SendALE(res);
                    break;
                }
                case 5:
                {
                    byte[] pack = new byte[data.Length - 9];
                    Buffer.BlockCopy(data, 1, pack, 0, pack.Length);
                    HandleMessage(new ETCSVariables(pack));
                    break;
                }
                case 8:
                {
                    Active = false;
                    break;
                }
            }
        }
        System.UInt16 crc16(byte[] ptr, int count)
        {
            System.UInt16 crc = 0xFFFF;
            for (int i=0; i<count; i++)
            {
                crc = (System.UInt16)(crc ^ (((System.UInt16)ptr[i]) << 8));
                for (int j=0; j<8; j++) {
                    if ((crc & 0x8000) != 0)
                        crc = (System.UInt16)(crc << 1 ^ 0x1021);
                    else
                        crc = (System.UInt16)(crc << 1);
                }
            }
            return crc;
        }
        int SeqNumberTx;
        void SendALE(byte[] data)
        {
            int type = (data[0]>>1)&15;
            byte[] ale;
            int sz = type == 2 ? 14 : 10;
            ale = new byte[data.Length + sz];
            Buffer.BlockCopy(data, 0, ale, sz, data.Length);
            ale[0] = (byte)(((ale.Length-2)>>8) & 255);
            ale[1] = (byte)((ale.Length-2) & 255);
            ale[2] = 0;
            ale[3] = 16;
            if (type == 2) SeqNumberTx = 0;
            ale[4] = (byte)((SeqNumberTx>>8)&255);
            ale[5] = (byte)(SeqNumberTx&255);
            SeqNumberTx++;
            ale[6] = 1;
            switch (type)
            {
                case 1:
                case 2:
                    ale[7] = (byte)type;
                    break;
                case 8:
                    ale[7] = (byte)4;
                    break;
                default:
                    ale[7] = (byte)3;
                    break;
            }
            int checksum = crc16(ale, 8);
            ale[8] = (byte)((checksum>>8)&255);
            ale[9] = (byte)(checksum & 255);

            if (type == 2)
            {
                ale[10] = 1;
                int rbc=(RBC_session.NID_C<<14)|RBC_session.NID_RBC;
                for (int i=0; i<3; i++)
                {
                    ale[11+i] = (byte)((rbc>>((2-i)*8))&255);
                }
            }

            Console.WriteLine("Writing: ");
            for (int i = 0; i < ale.Length; i++)
            {
                Console.Write(ale[i]);
                Console.Write(" ");
            }
            Console.WriteLine();

            try
            {
                s.Send(ale);
            }
            catch (Exception e)
            {
                Active = false;
                if (session != null) session.Connection = null;
                Console.WriteLine("==============RBC error==============");
                Console.WriteLine(e.StackTrace);
                Console.WriteLine("==============RBC error==============");
            }
        }
        void HandleMessage(ETCSVariables msg)
        {
            int nid_message = (int)msg.Access(0, 8);
            int t_train = (int)msg.Access(18, 32);
            int nid_engine = (int)msg.Access(50, 24);
            if (session == null)
            {
                sessions.TryGetValue(nid_engine, out RBC_session s);
                if (s != null && (s.Established || s.Establishing) && nid_message != 155)
                {
                    session = s;
                    session.Connection = this;
                }
                else
                {
                    if (s != null) s.Established = s.Establishing = false;
                    session = new RBC_session(this, nid_engine);
                    sessions[nid_engine] = session;
                }
            }
            if (session.Established || session.Establishing) session.Handle(msg);
            else Active = false;
        }
        public void Update()
        {
            if (ReadTask == null)
            {
                ReadTask = ReadTCP();
            }
            if (ReadTask.IsFaulted) Active = false;
            else if (ReadTask.IsCompleted)
            {
                byte[] data = ReadTask.Result;
                if (data == null)
                {
                    Active = false;
                    return;
                }
                HandleALE(data);
                ReadTask = ReadTCP();
            }
            if (!Active) State = ConnectionState.IDLE;
        }
        public void Dispose()
        {
            s.Close();
        }
        public void Send(ETCSVariables msg)
        {
            byte[] arr = msg.ToArray();
            LastSentMessage = DateTime.UtcNow;
            byte[] data = new byte[arr.Length+9];
            Buffer.BlockCopy(arr, 0, data, 1, arr.Length);
            data[0] = (byte)((5<<1) | 1);
            SendALE(data);
        }
    }
}
