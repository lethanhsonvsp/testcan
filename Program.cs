using System;
using System.Linq;
using System.Threading;
using System.Diagnostics;
using System.Collections.Concurrent;
using SocketCANSharp;
using SocketCANSharp.Network;

namespace testcan
{
    #region Enums
    public enum CiA402State
    {
        NotReadyToSwitchOn = 0, SwitchOnDisabled = 1, ReadyToSwitchOn = 2,
        SwitchedOn = 3, OperationEnabled = 4, QuickStopActive = 5,
        FaultReactionActive = 6, Fault = 7
    }
    public enum OperationMode : sbyte
    {
        ProfilePosition = 1, VelocityMode = 2, ProfileVelocity = 3,
        Homing = 6, CyclicSynchronousPosition = 8,
        CyclicSynchronousVelocity = 9, CyclicSynchronousTorque = 10
    }
    #endregion
    #region PDO Data Structures
    public struct TPDO1Data
    {
        public ushort StatusWord;
        public int ActualPosition;
        public short ActualTorque;
        public TPDO1Data(byte[] data)
        {
            StatusWord = 0; ActualPosition = 0; ActualTorque = 0;
            if (data.Length >= 2) StatusWord = (ushort)(data[0] | (data[1] << 8));
            if (data.Length >= 6) ActualPosition = BitConverter.ToInt32(data, 2);
            if (data.Length >= 8) ActualTorque = BitConverter.ToInt16(data, 6);
        }
        public readonly bool IsOperationEnabled() => (StatusWord & 0x6F) == 0x27;
        public readonly bool HasFault() => (StatusWord & 0x08) != 0;
    }
    public struct TPDO2Data
    {
        public int ActualVelocity;
        public byte ModesOfOperationDisplay;
        public TPDO2Data(byte[] data)
        {
            if (data.Length >= 5)
            {
                ActualVelocity = BitConverter.ToInt32(data, 0);
                ModesOfOperationDisplay = data[4];
            }
            else { ActualVelocity = 0; ModesOfOperationDisplay = 0; }
        }
    }
    #endregion
    #region UbuntuCANInterface - With SocketCANSharp
    public class UbuntuCANInterface
    {
        private string canInterface = "";
        private bool isConnected = false;
        private readonly object sdoLock = new();
        private readonly object nodeLock = new();
        private CanNetworkInterface? canInterfaceHandle;
        private RawCanSocket? canSocket;
        private readonly ConcurrentQueue<CanFrame> canFrames = new();
        private volatile bool isMonitoring = false;
        private const int MAX_QUEUE_SIZE = 500;
        private readonly ConcurrentDictionary<byte, TPDO1Data> latestTPDO1 = new();
        private readonly ConcurrentDictionary<byte, TPDO2Data> latestTPDO2 = new();
        private readonly ConcurrentDictionary<byte, DateTime> lastTPDO1Update = new();
        private readonly ConcurrentDictionary<byte, DateTime> lastTPDO2Update = new();

        public event EventHandler<string>? CANFrameReceived;
        public event EventHandler<(byte nodeId, TPDO1Data data)>? TPDO1Received;
        public event EventHandler<(byte nodeId, TPDO2Data data)>? TPDO2Received;

        public string GetInterfaceName() => canInterface;
        public TPDO1Data GetLatestTPDO1(byte nodeId) =>
            latestTPDO1.TryGetValue(nodeId, out var data) ? data : new TPDO1Data();
        public TPDO2Data GetLatestTPDO2(byte nodeId) =>
            latestTPDO2.TryGetValue(nodeId, out var data) ? data : new TPDO2Data();
        public DateTime GetLastTPDO1Time(byte nodeId) =>
            lastTPDO1Update.TryGetValue(nodeId, out var time) ? time : DateTime.MinValue;
        public DateTime GetLastTPDO2Time(byte nodeId) =>
            lastTPDO2Update.TryGetValue(nodeId, out var time) ? time : DateTime.MinValue;

        public bool Connect(string interfaceName, int baudrate)
        {
            try
            {
                canInterface = interfaceName;
                Console.WriteLine($"Thiết lập giao diện CAN {interfaceName} với baudrate {baudrate}");
                string result = ExecuteCommand("sudo /usr/local/bin/setup_can.sh");
                if (result.Contains("error", StringComparison.OrdinalIgnoreCase))
                {
                    Console.WriteLine($"Lỗi thiết lập giao diện CAN: {result}");
                    return false;
                }
                canSocket = new RawCanSocket();
                canInterfaceHandle = CanNetworkInterface.GetAllInterfaces(true).FirstOrDefault(i => i.Name == interfaceName);
                if (canInterfaceHandle == null)
                {
                    Console.WriteLine($"CAN: Interface {interfaceName} not found");
                    return false;
                }
                canSocket.Bind(canInterfaceHandle);
                isConnected = true;
                Console.WriteLine($"Kết nối thành công {interfaceName}");
                StartCANMonitoring();
                Thread.Sleep(200);
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Lỗi kết nối CAN: {ex.Message}");
                return false;
            }
        }

        public bool SendNMT(byte command, byte targetNodeId)
        {
            if (!isConnected || canSocket == null) return false;
            try
            {
                uint cobId = 0x000;
                byte[] data = new byte[8]; // Đảm bảo mảng 8 byte
                data[0] = command;
                data[1] = targetNodeId;
                var frame = new CanFrame
                {
                    CanId = cobId,
                    Data = data,
                    Length = 2
                };
                canSocket.Write(frame);
                string frameData = BitConverter.ToString(data, 0, 2).Replace("-", "");
                Console.WriteLine($"NMT command sent: 0x{command:X2} to node {targetNodeId}");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Lỗi SendNMT: {ex.Message}");
                return false;
            }
        }

        public bool SendRPDO2(byte nodeId, int targetVelocity, sbyte modesOfOperation)
        {
            if (!isConnected || canSocket == null) return false;
            try
            {
                uint cobId = (uint)(0x300 + nodeId);
                byte[] data = new byte[8];
                byte[] velBytes = BitConverter.GetBytes(targetVelocity);
                Array.Copy(velBytes, 0, data, 0, 4);
                data[4] = (byte)modesOfOperation;
                var frame = new CanFrame
                {
                    CanId = cobId,
                    Data = data,
                    Length = 5
                };
                canSocket.Write(frame);
                string frameData = BitConverter.ToString(data, 0, 5).Replace("-", "");
                Console.WriteLine($"RPDO2 sent: {cobId:X3}#{frameData}");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Lỗi SendRPDO2: {ex.Message}");
                return false;
            }
        }

        private void StartCANMonitoring()
        {
            if (isMonitoring || canSocket == null) return;
            isMonitoring = true;
            Task.Run(() =>
            {
                try
                {
                    while (isMonitoring)
                    {
                        CanFrame frame;
                        canSocket.Read(out frame);
                        while (canFrames.Count > MAX_QUEUE_SIZE)
                            canFrames.TryDequeue(out _);
                        canFrames.Enqueue(frame);
                        string frameStr = FrameToString(frame);
                        CANFrameReceived?.Invoke(this, frameStr);
                        ProcessPDOMessage(frame);
                        Thread.Sleep(1);
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Lỗi monitor CAN: {ex.Message}");
                    isMonitoring = false;
                }
            });
        }

        private void ProcessPDOMessage(CanFrame frame)
        {
            uint cobId = frame.CanId;
            if (cobId >= 0x180 && cobId <= 0x1FF)
            {
                byte nodeId = (byte)(cobId - 0x180);
                var tpdo1 = new TPDO1Data(frame.Data);
                latestTPDO1[nodeId] = tpdo1;
                lastTPDO1Update[nodeId] = DateTime.Now;
                TPDO1Received?.Invoke(this, (nodeId, tpdo1));
            }
            else if (cobId >= 0x280 && cobId <= 0x2FF)
            {
                byte nodeId = (byte)(cobId - 0x280);
                var tpdo2 = new TPDO2Data(frame.Data);
                latestTPDO2[nodeId] = tpdo2;
                lastTPDO2Update[nodeId] = DateTime.Now;
                TPDO2Received?.Invoke(this, (nodeId, tpdo2));
            }
        }

        public void Disconnect()
        {
            if (isConnected)
            {
                isMonitoring = false;
                try
                {
                    canSocket?.Close();
                    canSocket?.Dispose();
                    ExecuteCommand($"sudo ip link set {canInterface} down");
                    Console.WriteLine($"Ngắt kết nối CAN interface {canInterface}");
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Lỗi ngắt kết nối CAN: {ex.Message}");
                }
                isConnected = false;
            }
        }

        public bool WriteSDO(byte nodeId, ushort index, byte subindex, uint data, byte dataSize)
        {
            if (!isConnected || canSocket == null) return false;
            lock (nodeLock)
            {
                lock (sdoLock)
                {
                    try
                    {
                        while (canFrames.TryDequeue(out _)) { }
                        byte command = dataSize switch
                        {
                            1 => 0x2F,
                            2 => 0x2B,
                            4 => 0x23,
                            _ => throw new ArgumentException($"Kích thước dữ liệu không hỗ trợ: {dataSize}")
                        };
                        byte[] frameData = new byte[8];
                        frameData[0] = command;
                        frameData[1] = (byte)(index & 0xFF);
                        frameData[2] = (byte)(index >> 8);
                        frameData[3] = subindex;
                        byte[] dataBytes = BitConverter.GetBytes(data);
                        Array.Copy(dataBytes, 0, frameData, 4, Math.Min((int)dataSize, 4));
                        uint cobId = (uint)(0x600 + nodeId);
                        var frame = new CanFrame
                        {
                            CanId = cobId,
                            Data = frameData,
                            Length = (byte)frameData.Length
                        };
                        canSocket.Write(frame);
                        string frameDataStr = BitConverter.ToString(frameData, 0, frame.Length).Replace("-", "");
                        Console.WriteLine($"SDO Write: {cobId:X3}#{frameDataStr}");
                        return WaitForSDOResponse((uint)(0x580 + nodeId), true);
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"Lỗi WriteSDO: {ex.Message}");
                        return false;
                    }
                }
            }
        }

        public uint ReadSDO(byte nodeId, ushort index, byte subindex)
        {
            if (!isConnected || canSocket == null) return 0;
            lock (nodeLock)
            {
                lock (sdoLock)
                {
                    try
                    {
                        while (canFrames.TryDequeue(out _)) { }
                        byte[] frameData = new byte[8];
                        frameData[0] = 0x40;
                        frameData[1] = (byte)(index & 0xFF);
                        frameData[2] = (byte)(index >> 8);
                        frameData[3] = subindex;
                        uint cobId = (uint)(0x600 + nodeId);
                        var frame = new CanFrame
                        {
                            CanId = cobId,
                            Data = frameData,
                            Length = (byte)frameData.Length
                        };
                        canSocket.Write(frame);
                        string frameDataStr = BitConverter.ToString(frameData, 0, frame.Length).Replace("-", "");
                        Console.WriteLine($"SDO Read: {cobId:X3}#{frameDataStr}");
                        return WaitForSDOReadResponse((uint)(0x580 + nodeId));
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"Lỗi ReadSDO: {ex.Message}");
                        return 0;
                    }
                }
            }
        }

        private bool WaitForSDOResponse(uint expectedCOBID, bool isWrite, int timeoutMs = 2000)
        {
            var startTime = DateTime.Now;
            while ((DateTime.Now - startTime).TotalMilliseconds < timeoutMs)
            {
                if (canFrames.TryDequeue(out CanFrame frame))
                {
                    if (frame.CanId == expectedCOBID && frame.Length > 0)
                    {
                        try
                        {
                            byte responseCmd = frame.Data[0];
                            if (responseCmd == 0x80) return false;
                            if (isWrite && responseCmd == 0x60) return true;
                        }
                        catch { }
                    }
                }
                Thread.Sleep(1);
            }
            return false;
        }

        private uint WaitForSDOReadResponse(uint expectedCOBID, int timeoutMs = 2000)
        {
            var startTime = DateTime.Now;
            while ((DateTime.Now - startTime).TotalMilliseconds < timeoutMs)
            {
                if (canFrames.TryDequeue(out CanFrame frame))
                {
                    if (frame.CanId == expectedCOBID && frame.Length > 0)
                    {
                        try
                        {
                            byte responseCmd = frame.Data[0];
                            if (responseCmd == 0x80) return 0;
                            return responseCmd switch
                            {
                                0x4F => frame.Data[4],
                                0x4B => (uint)(frame.Data[4] | (frame.Data[5] << 8)),
                                0x43 => (uint)(frame.Data[4] | (frame.Data[5] << 8) | (frame.Data[6] << 16) | (frame.Data[7] << 24)),
                                _ => 0u
                            };
                        }
                        catch { }
                    }
                }
                Thread.Sleep(1);
            }
            return 0;
        }

        private static string FrameToString(CanFrame frame)
        {
            var dataHex = BitConverter.ToString(frame.Data, 0, frame.Length).Replace("-", "");
            return $"{frame.CanId:X3}#{dataHex}";
        }

        private static string ExecuteCommand(string command)
        {
            try
            {
                var process = new Process
                {
                    StartInfo = new ProcessStartInfo
                    {
                        FileName = "/bin/bash",
                        Arguments = $"-c \"{command}\"",
                        UseShellExecute = false,
                        RedirectStandardOutput = true,
                        RedirectStandardError = true,
                        CreateNoWindow = true
                    }
                };
                process.Start();
                string output = process.StandardOutput.ReadToEnd();
                string error = process.StandardError.ReadToEnd();
                process.WaitForExit();
                if (!string.IsNullOrEmpty(error) && !error.Contains("RTNETLINK"))
                    return error;
                return output;
            }
            catch (Exception ex)
            {
                return $"Lỗi: {ex.Message}";
            }
        }
    }
    #endregion
    #region CiA402Motor - PDO Mode
    public class CiA402Motor
    {
        private readonly UbuntuCANInterface canInterface;
        private readonly byte nodeId;
        private CiA402State currentState = CiA402State.NotReadyToSwitchOn;
        private bool usePDO = false;
        private const ushort CONTROL_WORD = 0x6040;
        private const ushort STATUS_WORD = 0x6041;
        private const ushort TARGET_VELOCITY = 0x60FF;
        private const ushort VELOCITY_ACTUAL = 0x606C;
        private const double ENCODER_RES = 10000.0;
        private const ushort POSITION_ACTUAL = 0x6064;
        private const ushort ERROR_CODE = 0x603F;

   
        public CiA402Motor(UbuntuCANInterface canInterface, byte nodeId)
        {
            this.canInterface = canInterface;
            this.nodeId = nodeId;
        }
        public void EnablePDOMode(bool enable)
        {
            usePDO = enable;
        }
        public bool ConfigurePDO()
        {
            try
            {
                canInterface.WriteSDO(nodeId, 0x1401, 1, 0x80000300 + nodeId, 4);
                canInterface.WriteSDO(nodeId, 0x1601, 0, 0, 1);
                canInterface.WriteSDO(nodeId, 0x1601, 1, 0x60FF0020, 4);
                canInterface.WriteSDO(nodeId, 0x1601, 2, 0x60600008, 4);
                canInterface.WriteSDO(nodeId, 0x1601, 0, 2, 1);
                canInterface.WriteSDO(nodeId, 0x1401, 1, (uint)(0x00000300 + nodeId), 4);
                canInterface.WriteSDO(nodeId, 0x1800, 1, 0x80000180 + nodeId, 4);
                canInterface.WriteSDO(nodeId, 0x1A00, 0, 0, 1);
                canInterface.WriteSDO(nodeId, 0x1A00, 1, 0x60410010, 4);
                canInterface.WriteSDO(nodeId, 0x1A00, 2, 0x60640020, 4);
                canInterface.WriteSDO(nodeId, 0x1A00, 3, 0x60770010, 4);
                canInterface.WriteSDO(nodeId, 0x1A00, 0, 3, 1);
                canInterface.WriteSDO(nodeId, 0x1800, 1, (uint)(0x00000180 + nodeId), 4);
                canInterface.WriteSDO(nodeId, 0x1801, 1, 0x80000280 + nodeId, 4);
                canInterface.WriteSDO(nodeId, 0x1A01, 0, 0, 1);
                canInterface.WriteSDO(nodeId, 0x1A01, 1, 0x606C0020, 4);
                canInterface.WriteSDO(nodeId, 0x1A01, 2, 0x60610008, 4);
                canInterface.WriteSDO(nodeId, 0x1A01, 0, 2, 1);
                canInterface.WriteSDO(nodeId, 0x1801, 1, (uint)(0x00000280 + nodeId), 4);
                Thread.Sleep(500);
                usePDO = true;
                Console.WriteLine($"PDO configured for node {nodeId}");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Lỗi ConfigurePDO node {nodeId}: {ex.Message}");
                return false;
            }
        }

        public bool Initialize()
        {
            try
            {
                UpdateState();
                if (currentState == CiA402State.Fault)
                {
                    ResetFault();
                    Thread.Sleep(500);
                    UpdateState();
                }
                if (!EnableOperation())
                {
                    Console.WriteLine($"Lỗi: Không thể kích hoạt Operation cho node {nodeId}");
                    return false;
                }
                canInterface.WriteSDO(nodeId, 0x6060, 0, (byte)OperationMode.CyclicSynchronousVelocity, 1);
                Thread.Sleep(100);
                canInterface.WriteSDO(nodeId, TARGET_VELOCITY, 0, 0, 4);
                Thread.Sleep(100);
                Console.WriteLine($"Initialized motor node {nodeId}");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Lỗi Initialize node {nodeId}: {ex.Message}");
                return false;
            }
        }

        public bool SetVelocityRpm(double rpm)
        {
            int countsPerSec = (int)(rpm * ENCODER_RES / 60.0);
            return SetVelocity(countsPerSec);
        }

        public double GetActualVelocityRpm()
        {
            int countsPerSec = GetActualVelocity();
            return countsPerSec * 60.0 / ENCODER_RES;
        }

        public bool Disable()
        {
            try
            {
                SetVelocity(0);
                Thread.Sleep(100);
                bool result = canInterface.WriteSDO(nodeId, CONTROL_WORD, 0, 0x07, 2);
                Console.WriteLine($"Disabled motor node {nodeId}: {result}");
                return result;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Lỗi Disable node {nodeId}: {ex.Message}");
                return false;
            }
        }

        private void UpdateState()
        {
            uint statusWord = usePDO ?
                canInterface.GetLatestTPDO1(nodeId).StatusWord :
                canInterface.ReadSDO(nodeId, STATUS_WORD, 0);
            currentState = DecodeState(statusWord);
        }

        private static CiA402State DecodeState(uint statusWord)
        {
            return (statusWord & 0x4F) switch
            {
                0x00 => CiA402State.NotReadyToSwitchOn,
                0x40 => CiA402State.SwitchOnDisabled,
                0x08 => CiA402State.Fault,
                0x0F => CiA402State.FaultReactionActive,
                _ => (statusWord & 0x6F) switch
                {
                    0x21 => CiA402State.ReadyToSwitchOn,
                    0x23 => CiA402State.SwitchedOn,
                    0x27 => CiA402State.OperationEnabled,
                    0x07 => CiA402State.QuickStopActive,
                    _ => CiA402State.NotReadyToSwitchOn
                }
            };
        }

        private bool ResetFault()
        {
            try
            {
                bool result = canInterface.WriteSDO(nodeId, CONTROL_WORD, 0, 0x80, 2);
                Console.WriteLine($"Reset fault node {nodeId}: {result}");
                return result;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Lỗi ResetFault node {nodeId}: {ex.Message}");
                return false;
            }
        }

        private bool EnableOperation()
        {
            for (int i = 0; i < 10; i++)
            {
                UpdateState();
                switch (currentState)
                {
                    case CiA402State.SwitchOnDisabled:
                        canInterface.WriteSDO(nodeId, CONTROL_WORD, 0, 0x06, 2);
                        Thread.Sleep(200);
                        break;
                    case CiA402State.ReadyToSwitchOn:
                        canInterface.WriteSDO(nodeId, CONTROL_WORD, 0, 0x07, 2);
                        Thread.Sleep(200);
                        break;
                    case CiA402State.SwitchedOn:
                        canInterface.WriteSDO(nodeId, CONTROL_WORD, 0, 0x0F, 2);
                        Thread.Sleep(200);
                        break;
                    case CiA402State.OperationEnabled:
                        return true;
                    case CiA402State.Fault:
                        ResetFault();
                        Thread.Sleep(500);
                        break;
                    default:
                        Thread.Sleep(200);
                        break;
                }
            }
            return false;
        }

        private bool SetVelocity(int targetVelocity)
        {
            try
            {
                if (usePDO)
                {
                    return canInterface.SendRPDO2(nodeId, targetVelocity, (sbyte)OperationMode.CyclicSynchronousVelocity);
                }
                else
                {
                    uint velocityData = targetVelocity < 0 ?
                        (uint)(targetVelocity + 0x100000000L) : (uint)targetVelocity;
                    canInterface.WriteSDO(nodeId, TARGET_VELOCITY, 0, velocityData, 4);
                    return canInterface.WriteSDO(nodeId, CONTROL_WORD, 0, 0x0F, 2);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Lỗi SetVelocity node {nodeId}: {ex.Message}");
                return false;
            }
        }

        private int GetActualVelocity()
        {
            try
            {
                if (usePDO)
                {
                    return canInterface.GetLatestTPDO2(nodeId).ActualVelocity;
                }
                else
                {
                    uint rawValue = canInterface.ReadSDO(nodeId, VELOCITY_ACTUAL, 0);
                    return rawValue > 0x7FFFFFFF ? (int)(rawValue - 0x100000000L) : (int)rawValue;
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Lỗi GetActualVelocity node {nodeId}: {ex.Message}");
                return 0;
            }
        }
        public int GetActualPosition()
        {
            try
            {
                if (usePDO)
                {
                    return canInterface.GetLatestTPDO1(nodeId).ActualPosition;
                }
                else
                {
                    uint rawValue = canInterface.ReadSDO(nodeId, POSITION_ACTUAL, 0);
                    return unchecked((int)rawValue);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Lỗi GetActualPosition node {nodeId}: {ex.Message}");
                return 0;
            }
        }

        public ushort GetErrorCode()
        {
            try
            {
                uint code = canInterface.ReadSDO(nodeId, ERROR_CODE, 0);
                return (ushort)(code & 0xFFFF);
            }
            catch
            {
                return 0;
            }
        }
    }
    #endregion
    #region TwoWheelRobot - Keyboard Control
    public class TwoWheelRobot
    {
        private readonly CiA402Motor left;
        private readonly CiA402Motor right;
        private const double MAX_RPM = 1500.0;
        private const double RPM_STEP = 100.0; // Tăng/giảm 100 RPM mỗi lần nhấn phím
        private volatile bool isRunning = false;
        private double lastLeftRpm = 0;
        private double lastRightRpm = 0;
        private DateTime lastUpdateTime = DateTime.Now;
        private const int UPDATE_INTERVAL_MS = 20;

        public TwoWheelRobot(CiA402Motor left, CiA402Motor right)
        {
            this.left = left;
            this.right = right;
        }

        public void Start()
        {
            isRunning = true;
        }

        public void Stop()
        {
            isRunning = false;
            SetMotorSpeeds(0, 0);
        }

        public void UpdateFromKeyboard(ConsoleKeyInfo keyInfo)
        {
            if (!isRunning) return;
            var now = DateTime.Now;
            if ((now - lastUpdateTime).TotalMilliseconds < UPDATE_INTERVAL_MS) return;
            lastUpdateTime = now;

            double speedChange = 0;
            switch (keyInfo.Key)
            {
                case ConsoleKey.UpArrow:
                    speedChange = RPM_STEP; // Tăng tốc tiến
                    break;
                case ConsoleKey.DownArrow:
                    speedChange = -RPM_STEP; // Tăng tốc lùi
                    break;
                case ConsoleKey.Spacebar:
                    speedChange = 0; // Dừng
                    SetMotorSpeeds(0, 0);
                    return;
                default:
                    return; // Bỏ qua các phím khác
            }

            double newRpm = Math.Max(-MAX_RPM, Math.Min(MAX_RPM, lastLeftRpm + speedChange));
            SetMotorSpeeds(newRpm, newRpm); // Cả hai động cơ cùng tốc độ
        }

        private void SetMotorSpeeds(double leftRpm, double rightRpm)
        {
            if (Math.Abs(leftRpm - lastLeftRpm) > 0.5)
            {
                left.SetVelocityRpm(-leftRpm);
                lastLeftRpm = leftRpm;
            }
            if (Math.Abs(rightRpm - lastRightRpm) > 0.5)
            {
                right.SetVelocityRpm(rightRpm);
                lastRightRpm = rightRpm;
            }
            Console.WriteLine($"Set speeds: Left={leftRpm:F1} RPM, Right={rightRpm:F1} RPM");
        }

        public string GetStatusString()
        {
            double leftVel = left.GetActualVelocityRpm();
            double rightVel = right.GetActualVelocityRpm();
            int leftPos = left.GetActualPosition();
            int rightPos = right.GetActualPosition();

            ushort leftErr = left.GetErrorCode();
            ushort rightErr = right.GetErrorCode();

            string errStr = (leftErr != 0 || rightErr != 0)
                ? $" | ERR L:{leftErr:X4} R:{rightErr:X4}"
                : "";

            return $"L:{leftVel,6:F1}rpm P:{leftPos,8} | R:{rightVel,6:F1}rpm P:{rightPos,8}{errStr}";
        }


        public (double left, double right) GetTargetSpeeds()
        {
            return (lastLeftRpm, lastRightRpm);
        }
    }
    #endregion
    #region Main Program
    class Program
    {
        static void Main(string[] args)
        {
            // Cấu hình terminal để đọc phím không cần nhấn Enter
            ConfigureConsole();

            string canInterface = "can1";
            int baudrate = 500000;
            byte leftNodeId = 1;
            byte rightNodeId = 2;

            var sharedCANInterface = new UbuntuCANInterface();
            try
            {
                if (!sharedCANInterface.Connect(canInterface, baudrate))
                {
                    Console.WriteLine("Không thể kết nối CAN interface");
                    return;
                }

                sharedCANInterface.SendNMT(0x81, leftNodeId);
                sharedCANInterface.SendNMT(0x81, rightNodeId);
                Thread.Sleep(1000);
                sharedCANInterface.SendNMT(0x01, leftNodeId);
                sharedCANInterface.SendNMT(0x01, rightNodeId);
                Thread.Sleep(500);

                var leftMotor = new CiA402Motor(sharedCANInterface, leftNodeId);
                if (!leftMotor.Initialize())
                {
                    Console.WriteLine("Không thể khởi tạo động cơ trái");
                    return;
                }
                if (!leftMotor.ConfigurePDO())
                {
                    Console.WriteLine("Không thể cấu hình PDO cho động cơ trái");
                    return;
                }
                leftMotor.EnablePDOMode(true);

                var rightMotor = new CiA402Motor(sharedCANInterface, rightNodeId);
                if (!rightMotor.Initialize())
                {
                    Console.WriteLine("Không thể khởi tạo động cơ phải");
                    return;
                }
                if (!rightMotor.ConfigurePDO())
                {
                    Console.WriteLine("Không thể cấu hình PDO cho động cơ phải");
                    return;
                }
                rightMotor.EnablePDOMode(true);

                var robot = new TwoWheelRobot(leftMotor, rightMotor);
                robot.Start();

                // Thread hiển thị trạng thái
                var statusThread = new Thread(() =>
                {
                    while (true)
                    {
                        try
                        {
                            Console.WriteLine(robot.GetStatusString());
                            Thread.Sleep(500);
                        }
                        catch { break; }
                    }
                })
                { IsBackground = true };
                statusThread.Start();

                // Đọc bàn phím
                while (true)
                {
                    if (Console.KeyAvailable)
                    {
                        var keyInfo = Console.ReadKey(true);
                        if (keyInfo.Key == ConsoleKey.Q || keyInfo.Key == ConsoleKey.Escape)
                        {
                            robot.Stop();
                            Thread.Sleep(500);
                            leftMotor.Disable();
                            rightMotor.Disable();
                            sharedCANInterface.Disconnect();
                            break;
                        }
                        robot.UpdateFromKeyboard(keyInfo);
                    }
                    Thread.Sleep(10);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Lỗi chương trình: {ex.Message}");
            }
            finally
            {
                try
                {
                    sharedCANInterface?.Disconnect();
                }
                catch { }
                RestoreConsole();
            }
        }

        private static void ConfigureConsole()
        {
            try
            {
                // Tắt chế độ echo và canonical để đọc phím trực tiếp
                ExecuteCommand("stty -echo -icanon");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Lỗi cấu hình console: {ex.Message}");
            }
        }

        private static void RestoreConsole()
        {
            try
            {
                // Khôi phục chế độ terminal
                ExecuteCommand("stty echo icanon");
            }
            catch { }
        }

        private static string ExecuteCommand(string command)
        {
            try
            {
                var process = new Process
                {
                    StartInfo = new ProcessStartInfo
                    {
                        FileName = "/bin/bash",
                        Arguments = $"-c \"{command}\"",
                        UseShellExecute = false,
                        RedirectStandardOutput = true,
                        RedirectStandardError = true,
                        CreateNoWindow = true
                    }
                };
                process.Start();
                string output = process.StandardOutput.ReadToEnd();
                string error = process.StandardError.ReadToEnd();
                process.WaitForExit();
                if (!string.IsNullOrEmpty(error) && !error.Contains("RTNETLINK"))
                    return error;
                return output;
            }
            catch (Exception ex)
            {
                return $"Lỗi: {ex.Message}";
            }
        }
    }
    #endregion
}