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
using Orts.MultiPlayer;
using Orts.Simulation.Signalling;
using ORTS.Common;
namespace RBC
{
    public class RBC
    {
        List<RBC_connection> Connections = new List<RBC_connection>();
        Task<Socket> ListenTask = null;
        Socket Listener;

        public bool AllowTAF = false;
        public bool AllowOSbeforeFS = false;
        public bool ExtendOSRoute = false;
        public bool L2TransitionAnnouncementWithMA = false;
        public bool L2TransitionOrder = true;
        public bool MABeforeTransition = false;
        public bool ERTMSRoutes = false;
        public bool SHModeProfile = true;
        public int NID_C;
        public int NID_RBC;

        public static Random random = new Random();
        public RBC(int NID_C, int NID_RBC, int port=0x7911)
        {
            Signals signals = MPManager.Simulator.Signals;
            RBC_session.Signals = signals;
            
            this.NID_C = NID_C;
            this.NID_RBC = NID_RBC;

            string section = string.Format("NID_C.{0}", NID_C);
            string ruleset = "ADIF_AV";
            LoadParameter(section, "Rules", ref ruleset);
            if (ruleset == "ADIF_CONV" || ruleset == "ADIF_AV")
            {
                AllowOSbeforeFS = false;
                AllowTAF = false;
                L2TransitionAnnouncementWithMA = true;
                L2TransitionOrder = true;
                MABeforeTransition = true;
            }
            else
            {
                AllowOSbeforeFS = true;
                AllowTAF = false;
                L2TransitionAnnouncementWithMA = true;
                L2TransitionOrder = false;
                MABeforeTransition = true;
            }
            LoadParameter(section,"OsBeforeFs", ref AllowOSbeforeFS);
            LoadParameter(section,"TAF", ref AllowTAF);
            LoadParameter(section,"ERTMSRoutes", ref ERTMSRoutes);
            LoadParameter(section,"ShProfile", ref SHModeProfile);
            
            IPEndPoint localEndPoint = new IPEndPoint(IPAddress.Any, port);
            Listener = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            Listener.Bind(localEndPoint);
            Listener.Listen(10);
        }
        public void LoadParameter<T>(string sectionName, string keyName, ref T value)
        {
            string file = Path.Combine(MPManager.Simulator.RoutePath, @"etcs.ini");
            if (File.Exists(file))
            {
                string buffer = new string('\0', 256);
                int length = NativeMethods.GetPrivateProfileString(sectionName, keyName, null, buffer, buffer.Length, file);

                if (length > 0)
                {
                    value = (T)Convert.ChangeType(buffer.Trim('\0').Trim(), typeof(T), System.Globalization.CultureInfo.InvariantCulture);
                }
            }
        }
        public void Update()
        {
            try
            {
                if (RBC_session.Signals == null)
                {
                    RBC_session.Signals = MPManager.Simulator.Signals;
                }

                if (ListenTask == null || ListenTask.IsCompleted)
                {
                    if (ListenTask != null)
                    {
                        Connections.Add(new RBC_connection(ListenTask.Result, this));
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
                Console.WriteLine("RBC error");
                Console.WriteLine(e.StackTrace);
            }
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
        readonly RBC Rbc;
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
        public RBC_connection(Socket s, RBC rbc)
        {
            this.s = s;
            Rbc = rbc;
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
            if (data[7] == 1) await Task.Delay((int)(RBC.random.Next(1000, 5000) / MPManager.Simulator.GameSpeed));
            else if (data[7] == 3) await Task.Delay((int)(RBC.random.Next(100, 1500) / MPManager.Simulator.GameSpeed));
            Console.Write("ERTMS RBC - Reading: ");
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
                    int rbc=(Rbc.NID_C<<14)|Rbc.NID_RBC;
                    for (int i=0; i<3; i++)
                    {
                        res[i+1] = (byte)((rbc>>((2-i)*8))&255);
                    }
                    res[4] = 1;

                    for (int i=0; i<4; i++)
                    {
                        res[5+i] = (byte)RBC.random.Next(0, 255);
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
        ushort crc16(byte[] ptr, int count)
        {
            ushort crc = 0xFFFF;
            for (int i=0; i<count; i++)
            {
                crc = (ushort)(crc ^ (((ushort)ptr[i]) << 8));
                for (int j=0; j<8; j++) {
                    if ((crc & 0x8000) != 0)
                        crc = (ushort)(crc << 1 ^ 0x1021);
                    else
                        crc = (ushort)(crc << 1);
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
                int rbc=(Rbc.NID_C<<14)|Rbc.NID_RBC;
                for (int i=0; i<3; i++)
                {
                    ale[11+i] = (byte)((rbc>>((2-i)*8))&255);
                }
            }

            Console.Write("ERTMS RBC - Writing: ");
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
                Console.WriteLine("RBC error");
                Console.WriteLine(e.StackTrace);
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
                    session = new RBC_session(this, Rbc, nid_engine);
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
