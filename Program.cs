using HidSharp;
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
            if (data.Length >= 2) StatusWord = (ushort)(data[0] | data[1] << 8);
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
    #region PS5 Controller
    public class PS5Controller
    {
        private HidDevice? device;
        private HidStream? stream;
        private volatile bool isRunning = false;
        private const int PS5_VENDOR_ID = 0x054C;
        private const int PS5_PRODUCT_ID = 0x0CE6;
        public struct ControllerState
        {
            public byte LeftStickX;
            public byte LeftStickY;
            public byte RightStickX;
            public byte RightStickY;
            public byte L2Trigger;
            public byte R2Trigger;
            public bool Square;
            public bool X;
            public bool Circle;
            public bool Triangle;
            public bool L1;
            public bool R1;
            public bool Share;
            public bool Options;
            public bool PS;
        }
        public event Action<ControllerState>? OnControllerUpdate;
        public bool Connect()
        {
            try
            {
                var devices = DeviceList.Local.GetHidDevices(PS5_VENDOR_ID, PS5_PRODUCT_ID);
                device = devices.FirstOrDefault();
                if (device == null)
                {
                    return false;
                }
                if (device.TryOpen(out stream))
                {
                    return true;
                }
                return false;
            }
            catch
            {
                return false;
            }
        }
        public void StartReading()
        {
            if (stream == null) return;
            isRunning = true;
            var readThread = new Thread(ReadControllerData) { IsBackground = true };
            readThread.Start();
        }
        private void ReadControllerData()
        {
            byte[] buffer = new byte[64];
            while (isRunning)
            {
                try
                {
                    int bytesRead = stream!.Read(buffer, 0, buffer.Length);
                    if (bytesRead > 0)
                    {
                        var state = ParseControllerData(buffer);
                        OnControllerUpdate?.Invoke(state);
                    }
                    Thread.Sleep(1);
                }
                catch
                {
                    break;
                }
            }
        }
        private static ControllerState ParseControllerData(byte[] data)
        {
            var state = new ControllerState();
            if (data.Length < 10) return state;
            byte rawLX = data[1];
            byte rawLY = data[2];
            byte rawRX = data[3];
            byte rawRY = data[4];
            state.LeftStickX = rawLX;
            state.LeftStickY = rawLY;
            state.RightStickX = rawRX;
            state.RightStickY = rawRY;
            state.LeftStickX = 127;
            state.RightStickY = 127;
            state.L2Trigger = data[5];
            state.R2Trigger = data[6];
            byte faceButtons = data[8];
            state.Square = (faceButtons & 0x10) != 0;
            state.X = (faceButtons & 0x20) != 0;
            state.Circle = (faceButtons & 0x40) != 0;
            state.Triangle = (faceButtons & 0x80) != 0;
            if (data.Length > 9)
            {
                byte shoulderButtons = data[9];
                state.L1 = (shoulderButtons & 0x01) != 0;
                state.R1 = (shoulderButtons & 0x02) != 0;
                state.Share = (shoulderButtons & 0x10) != 0;
                state.Options = (shoulderButtons & 0x20) != 0;
            }
            if (data.Length > 10)
            {
                state.PS = (data[10] & 0x01) != 0;
            }
            return state;
        }
        public void Stop()
        {
            isRunning = false;
            stream?.Close();
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

        public bool Connect(string interfaceName, int baudrate)
        {
            try
            {
                canInterface = interfaceName;
                ExecuteCommand($"sudo ip link set {interfaceName} down");
                ExecuteCommand($"sudo ip link set {interfaceName} type can bitrate {baudrate}");
                ExecuteCommand($"sudo ip link set {interfaceName} up");
                canSocket = new RawCanSocket();
                canInterfaceHandle = CanNetworkInterface.GetAllInterfaces(true).FirstOrDefault(i => i.Name == interfaceName);
                if (canInterfaceHandle == null)
                {
                    return false;
                }
                canSocket.Bind(canInterfaceHandle);
                isConnected = true;
                StartCANMonitoring();
                Thread.Sleep(200);
                return true;
            }
            catch
            {
                return false;
            }
        }

        public bool SendNMT(byte command, byte targetNodeId)
        {
            if (!isConnected || canSocket == null) return false;
            try
            {
                uint cobId = 0x000;
                byte[] data = [command, targetNodeId];

                var frame = new CanFrame
                {
                    CanId = cobId,
                    Data = data,
                    Length = (byte)data.Length
                };
                canSocket.Write(frame);
                return true;
            }
            catch
            {
                return false;
            }
        }

        public bool SendRPDO2(byte nodeId, int targetVelocity, sbyte modesOfOperation)
        {
            if (!isConnected || canSocket == null) return false;
            try
            {
                uint cobId = (uint)(0x300 + nodeId);
                byte[] data = new byte[5];
                byte[] velBytes = BitConverter.GetBytes(targetVelocity);
                Array.Copy(velBytes, 0, data, 0, 4);
                data[4] = (byte)modesOfOperation;
                // Fixed: Use object initializer instead of non-existent constructor
                var frame = new CanFrame
                {
                    CanId = cobId,
                    Data = data,
                    Length = (byte)data.Length
                };
                canSocket.Write(frame);
                return true;
            }
            catch
            {
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
                        CANFrameReceived?.Invoke(this, FrameToString(frame));
                        ProcessPDOMessage(frame);
                        Thread.Sleep(1);
                    }
                }
                catch
                {
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
                }
                catch { }
                ExecuteCommand($"sudo ip link set {canInterface} down");
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
                            _ => throw new ArgumentException()
                        };
                        byte[] frameData = new byte[8];
                        frameData[0] = command;
                        frameData[1] = (byte)(index & 0xFF);
                        frameData[2] = (byte)(index >> 8);
                        frameData[3] = subindex;
                        byte[] dataBytes = BitConverter.GetBytes(data);
                        Array.Copy(dataBytes, 0, frameData, 4, Math.Min((int)dataSize, 4));
                        uint cobId = (uint)(0x600 + nodeId);
                        // Replace this line in UbuntuCANInterface.WriteSDO and ReadSDO:
                        // var frame = new CanFrame(cobId, frameData.Length, frameData);
                        // With the following:
                        var frame = new CanFrame
                        {
                            CanId = cobId,
                            Data = frameData,
                            Length = (byte)frameData.Length
                        };
                        canSocket.Write(frame);
                        return WaitForSDOResponse((uint)(0x580 + nodeId), true);
                    }
                    catch
                    {
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
                        // Replace this line in UbuntuCANInterface.WriteSDO and ReadSDO:
                        // var frame = new CanFrame(cobId, frameData.Length, frameData);
                        // With the following:
                        var frame = new CanFrame
                        {
                            CanId = cobId,
                            Data = frameData,
                            Length = (byte)frameData.Length
                        };
                        canSocket.Write(frame);
                        return WaitForSDOReadResponse((uint)(0x580 + nodeId));
                    }
                    catch
                    {
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
                    // Replace all instances of 'frame.Dlc' with 'frame.Length'
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
                                0x4B => (uint)(frame.Data[4] | frame.Data[5] << 8),
                                0x43 => (uint)(frame.Data[4] | frame.Data[5] << 8 | frame.Data[6] << 16 | frame.Data[7] << 24),
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
                return string.IsNullOrEmpty(error) || error.Contains("RTNETLINK") ? output : error;
            }
            catch
            {
                return "";
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

        public CiA402Motor(UbuntuCANInterface canInterface, byte nodeId)
        {
            this.canInterface = canInterface;
            this.nodeId = nodeId;
        }

        public bool ConfigurePDO()
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
            return true;
        }

        public void EnablePDOMode(bool enable)
        {
            usePDO = enable;
        }

        public bool Initialize()
        {
            UpdateState();
            if (currentState == CiA402State.Fault)
            {
                ResetFault();
                Thread.Sleep(500);
                UpdateState();
            }
            if (!EnableOperation())
                return false;
            canInterface.WriteSDO(nodeId, 0x6060, 0, (byte)OperationMode.CyclicSynchronousVelocity, 1);
            Thread.Sleep(100);
            canInterface.WriteSDO(nodeId, TARGET_VELOCITY, 0, 0, 4);
            Thread.Sleep(100);
            return true;
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
            SetVelocity(0);
            Thread.Sleep(100);
            return canInterface.WriteSDO(nodeId, CONTROL_WORD, 0, 0x07, 2);
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
            return canInterface.WriteSDO(nodeId, CONTROL_WORD, 0, 0x80, 2);
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

        private int GetActualVelocity()
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
    }
    #endregion
    #region TwoWheelRobot - PDO Optimized
    public class TwoWheelRobot
    {
        private readonly CiA402Motor left;
        private readonly CiA402Motor right;
        private const double MAX_RPM = 1500.0;
        private const double DEADZONE = 35.0;
        private const double JOY_CENTER = 127.5;
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

        public void UpdateFromController(PS5Controller.ControllerState state)
        {
            if (!isRunning) return;
            if (!state.R1)
            {
                if (Math.Abs(lastLeftRpm) > 0.1 || Math.Abs(lastRightRpm) > 0.1)
                {
                    SetMotorSpeeds(0, 0);
                }
                return;
            }
            var now = DateTime.Now;
            if ((now - lastUpdateTime).TotalMilliseconds < UPDATE_INTERVAL_MS) return;
            lastUpdateTime = now;
            double forward = JOY_CENTER - state.LeftStickY;
            double turn = state.RightStickX - JOY_CENTER;
            if (Math.Abs(forward) < DEADZONE) forward = 0;
            if (Math.Abs(turn) < DEADZONE) turn = 0;
            forward = Math.Max(-1.0, Math.Min(1.0, forward / JOY_CENTER));
            turn = Math.Max(-1.0, Math.Min(1.0, turn / JOY_CENTER));
            double leftSpeed, rightSpeed;
            if (Math.Abs(forward) >= Math.Abs(turn))
            {
                leftSpeed = forward;
                rightSpeed = forward;
            }
            else
            {
                leftSpeed = turn;
                rightSpeed = -turn;
            }
            leftSpeed = Math.Max(-1.0, Math.Min(1.0, leftSpeed));
            rightSpeed = Math.Max(-1.0, Math.Min(1.0, rightSpeed));
            double leftRpm = leftSpeed * MAX_RPM;
            double rightRpm = rightSpeed * MAX_RPM;
            SetMotorSpeeds(leftRpm, rightRpm);
        }

        private void SetMotorSpeeds(double leftRpm, double rightRpm)
        {
            if (Math.Abs(leftRpm - lastLeftRpm) > 0.5)
            {
                left.SetVelocityRpm(leftRpm);
                lastLeftRpm = leftRpm;
            }
            if (Math.Abs(rightRpm - lastRightRpm) > 0.5)
            {
                right.SetVelocityRpm(rightRpm);
                lastRightRpm = rightRpm;
            }
        }

        public string GetStatusString()
        {
            double leftVel = left.GetActualVelocityRpm();
            double rightVel = right.GetActualVelocityRpm();
            return $"L: {leftVel,6:F1} | R: {rightVel,6:F1}";
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
            ArgumentNullException.ThrowIfNull(args);
            string canInterface = "can0";
            int baudrate = 500000;
            byte leftNodeId = 1;
            byte rightNodeId = 2;
            var sharedCANInterface = new UbuntuCANInterface();
            try
            {
                if (!sharedCANInterface.Connect(canInterface, baudrate))
                {
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
                    return;
                }
                if (!leftMotor.ConfigurePDO())
                {
                    return;
                }
                leftMotor.EnablePDOMode(true);
                var rightMotor = new CiA402Motor(sharedCANInterface, rightNodeId);
                if (!rightMotor.Initialize())
                {
                    return;
                }
                if (!rightMotor.ConfigurePDO())
                {
                    return;
                }
                rightMotor.EnablePDOMode(true);
                var ps5Controller = new PS5Controller();
                if (!ps5Controller.Connect())
                {
                    return;
                }
                var robot = new TwoWheelRobot(leftMotor, rightMotor);
                robot.Start();
                ps5Controller.OnControllerUpdate += (state) =>
                {
                    try
                    {
                        robot.UpdateFromController(state);
                        if (state.PS)
                        {
                            robot.Stop();
                            Thread.Sleep(500);
                            leftMotor.Disable();
                            rightMotor.Disable();
                            sharedCANInterface.Disconnect();
                            Environment.Exit(0);
                        }
                    }
                    catch { }
                };
                ps5Controller.StartReading();
                var statusThread = new Thread(() =>
                {
                    while (true)
                    {
                        try
                        {
                            robot.GetStatusString();
                            Thread.Sleep(500);
                        }
                        catch { }
                    }
                })
                { IsBackground = true };
                statusThread.Start();
                while (true)
                {
                    Thread.Sleep(100);
                }
            }
            catch
            {
            }
            finally
            {
                try
                {
                    sharedCANInterface?.Disconnect();
                }
                catch { }
            }
        }
    }
    #endregion
}