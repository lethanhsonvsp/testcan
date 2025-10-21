using SocketCANSharp;
using SocketCANSharp.Network;
using System;
using System.Linq;

class Program
{
    static void Main()
    {
        // Lấy interface (ví dụ: vcan0)
        CanNetworkInterface vcan0 = CanNetworkInterface.GetAllInterfaces(true)
            .First(iface => iface.Name.Equals("can0"));

        // Tạo và kết nối socket
        using var rawCanSocket = new RawCanSocket();
        rawCanSocket.Bind(vcan0);  // Bind đến interface

        // Ghi dữ liệu (Write)
        CanFrame writeFrame = new(0x123, [0x45, 0x67, 0x89, 0xab, 0xcd, 0xef]);
        int bytesWritten = rawCanSocket.Write(writeFrame);
        Console.WriteLine($"Đã ghi {bytesWritten} bytes.");

        // Đọc dữ liệu (Read)
        if (rawCanSocket.Read(out CanFrame readFrame) > 0)
        {
            Console.WriteLine($"Đã đọc frame ID: {readFrame.CanId}, Data: {BitConverter.ToString(readFrame.Data)}");
        }
    }
}