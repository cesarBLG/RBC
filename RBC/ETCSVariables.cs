using System;
using System.Collections.Generic;
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
}