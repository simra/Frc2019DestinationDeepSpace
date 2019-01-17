/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package trclib;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * This class implements a platform independent pixy camera. This class is intended to be extended by a platform
 * dependent pixy class which provides the abstract methods required by this class. This class provides the parser
 * to read and parse the object block from the pixy camera. It also provides access to the last detected objects
 * reported by the pixy camera asynchronously.
 */
public abstract class TrcPixy2Cam implements TrcNotifier.Receiver
{
    protected static final String moduleName = "TrcPixy2Cam";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;


    // Packet types: 
    private static final byte PIXY_SYNC_1                       =(byte)0xc1;
    private static final byte PIXY_SYNC_SEND_0                  =(byte)0xae; // 174,193
    private static final byte PIXY_SYNC_RECV_0                  =(byte)0xaf; // 175,193

    // Command types:
    private static final byte PIXY_CMD_GETVERSION               =(byte)0x0e; // 14
    private static final byte PIXY_CMD_GET_RESOLUTION           =(byte)0x0c; // 12
    private static final byte PIXY_CMD_SET_BRIGHTNESS           =(byte)0x10; // 16
    private static final byte PIXY_CMD_SET_SERVOS               =(byte)0x12; // 18
    private static final byte PIXY_CMD_SET_LED                  =(byte)0x14; // 20
    private static final byte PIXY_CMD_SET_LAMP                 =(byte)0x16; // 22
    private static final byte PIXY_CMD_GET_FPS                  =(byte)0x18; // 24
    private static final byte PIXY_CMD_GET_BLOCKS               =(byte)0x20; // 32
    private static final byte PIXY_CMD_GET_MAIN_FEATURES        =(byte)0x30; // 48
    private static final byte PIXY_CMD_SET_MODE                 =(byte)0x36; // 54
    private static final byte PIXY_CMD_SET_NEXT_TURN            =(byte)0x3a; // 58
    private static final byte PIXY_CMD_SET_DEFAULT_TURN         =(byte)0x3c; // 60
    private static final byte PIXY_CMD_SET_VECTOR               =(byte)0x38; // 56
    private static final byte PIXY_CMD_REVERSE_VECTOR           =(byte)0x3e; // 62
    private static final byte PIXY_CMD_GET_RGB                  =(byte)0x70; // 112
    
    // Response types
    private static final byte PIXY_RSP_GETVERSION               =(byte)0x0f; // 15
    private static final byte PIXY_RSP_GET_RESOLUTION           =(byte)0x0d; // 13
    private static final byte PIXY_RSP_SET_BRIGHTNESS           =(byte)0x11; // 17
    private static final byte PIXY_RSP_SET_SERVOS               =(byte)0x13; // 19
    private static final byte PIXY_RSP_SET_LED                  =(byte)0x15; // 21
    private static final byte PIXY_RSP_SET_LAMP                 =(byte)0x17; // 23
    private static final byte PIXY_RSP_GET_FPS                  =(byte)0x19; // 25
    private static final byte PIXY_RSP_GET_BLOCKS               =(byte)0x21; // 33
    private static final byte PIXY_RSP_GET_MAIN_FEATURES        =(byte)0x31; // 49
    private static final byte PIXY_RSP_GENERIC                  =(byte)0x01; // 01 
    

    /**
     * This method issues an asynchronous read of the specified number of bytes from the device.
     *
     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
     * @param length specifies the number of bytes to read.
     */
    public abstract void asyncReadData(RequestTag requestTag, int length);

    /**
     * This method writes the data buffer to the device asynchronously.
     *
     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
     * @param data specifies the data buffer.
     */
    public abstract void asyncWriteData(RequestTag requestTag, byte[] data);

    /**
     * This class implements the pixy camera object block communication protocol. 
     * rsim: TODO: verify that this is unchanged.
     */
    public class ObjectBlock
    {
        public int sync;
        public int checksum;
        public int signature;
        public int centerX;
        public int centerY;
        public int width;
        public int height;
        public int angle;

        public String toString()
        {
            return String.format(
                "sync=0x%04x, chksum=0x%04x, sig=%d, centerX=%3d, centerY=%3d, width=%3d, height=%3d, angle=%3d",
                sync, checksum, signature, centerX, centerY, width, height, angle);
        }
    }   //class ObjectBlock

    /**
     * This is used identify the request type.
     */
    public static enum RequestTag
    {
        SYNC,
        ALIGN,
        CHECKSUM,
        NORMAL_BLOCK,
        COLOR_CODE_BLOCK
    }   //enum RequestTag

    private final String instanceName;
    private final boolean msbFirst;
    private ArrayList<ObjectBlock> objects = new ArrayList<>();
    private ObjectBlock[] detectedObjects = null;
    private ObjectBlock currBlock = null;
    private int runningChecksum = 0;
    private boolean started = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param msbFirst specifies true if a word has MSB first.
     */
    public TrcPixy2Cam(final String instanceName, boolean msbFirst)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.msbFirst = msbFirst;
    }   //TrcPixyCam

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method starts the pixy camera by queuing the initial read request if not already.
     */
    public void start()
    {
        if (!started)
        {
            started = true;
            //asyncReadData(RequestTag.SYNC, 2);  // rsim: Removing as it's not clear this is needed for pixy2
        }
    }   //start

    /**
     * This method writes the data to the device one byte at a time.
     *
     * @param data specifies the buffer containing the data to be written to the device.
     */
    public void asyncWriteBytes(byte[] data)
    {
        byte[] byteData = new byte[1];

        for (int i = 0; i < data.length; i++)
        {
            byteData[0] = data[i];
            asyncWriteData(null, byteData);
        }
    }   //asyncWriteBytes

    
    /**
     * Allocate a packet. Use this to efficiently prep a packet for sending. len should *not* include the header bytes.
     */
    private byte[] allocPacket(byte command, byte[] payload) 
    {
        int bytesToAlloc=4;
        if (payload!=null) {
            bytesToAlloc+=payload.length;
        }
        byte[] result= new byte[bytesToAlloc];
        result[0]=PIXY_SYNC_SEND_0;
        result[1]=PIXY_SYNC_1;
        result[2]=command;
        result[3]=bytesToAlloc-4;
        if (payload!=null) {
            System.arraycopy(payload,0,result,4,payload.length);
        }
        return result;
    }

    public void getVersion() 
    {
        asyncWriteData(null, allocPacket(PIXY_CMD_GETVERSION, null));
    }

    public void getResolution()
    {
        asyncWriteData(null, allocPacket(PIXY_CMD_GET_RESOLUTION, new byte[] {0}));    
    }

    public void setCameraBrightness(byte brightness)
    {
        asyncWriteData(null, allocPacket(PIXY_CMD_SET_BRIGHTNESS, new byte[] {brightness}));
    }

    // s0 and s1 can range from 0 to 511
    public void setServos(short s0, short s1) 
    {
        asyncWriteData(null, allocPacket(PIXY_CMD_SET_SERVOS, 
            new byte[] {
                (byte)(s0&0xff), 
                (byte)(s0>>8), 
                (byte)(s1&0xff), 
                (byte)(s1>>8)
            }));
    }

    public void setLED(byte red, byte green, byte blue) 
    {
        asyncWriteData(null, allocPacket(PIXY_CMD_SET_LED, 
            new byte[] { red, green, blue}));
    }

    // upper and lower can each be 0 or 1
    public void setLamp(byte upper, byte lower) 
    {
        asyncWriteData(null, allocPacket(PIXY_CMD_SET_LAMP,
            new byte[] { upper, lower}));
    }

    public void getFps() 
    {
        asyncWriteData(null, allocPacket(PIXY_CMD_GET_FPS, null));
    }

    // GetBlocks

    
    /**
     * This method sets the LED to the specified color.
     *
     * @param red specifies the red value.
     * @param green specifies the green value.
     * @param blue specifies the blue value.
     */
    public void setLED(byte red, byte green, byte blue)
    {
        final String funcName = "setLED";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "red=%d,green=%d,blue=%d", red, green, blue);
        }

        byte[] data = new byte[5];
        data[0] = 0x00;
        data[1] = PIXY_CMD_SET_LED;
        data[2] = red;
        data[3] = green;
        data[4] = blue;

        asyncWriteData(null, data);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setLED

    /**
     * This method sets the camera brightness.
     *
     * @param brightness specifies the brightness value.
     */
    public void setBrightness(byte brightness)
    {
        final String funcName = "setBrightness";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "brightness=%d", brightness);
        }

        byte[] data = new byte[3];
        data[0] = 0x00;
        data[1] = PIXY_CMD_SET_BRIGHTNESS;
        data[2] = brightness;

        asyncWriteData(null, data);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setBrightness

    /**
     * This method sets the pan and tilt servo positions.
     * @param pan specifies the pan position between 0 and 1000.
     * @param tilt specifies the tilt position between 0 and 1000.
     */
    public void setPanTilt(int pan, int tilt)
    {
        final String funcName = "setPanTilt";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pan=%d,tilt=%d", pan, tilt);
        }

        if (pan < 0 || pan > 1000 || tilt < 0 || tilt > 1000)
        {
            throw new IllegalArgumentException("Invalid pan/tilt range.");
        }

        byte[] data = new byte[6];
        data[0] = 0x00;
        data[1] = PIXY_CMD_SET_PAN_TILT;
        data[2] = (byte)(pan & 0xff);
        data[3] = (byte)(pan >> 8);
        data[4] = (byte)(tilt & 0xff);
        data[5] = (byte)(tilt >> 8);

        asyncWriteData(null, data);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPanTilt

    /**
     * This method returns an array of detected object blocks.
     *
     * @return array of detected object blocks, can be null if no object detected or result of the next frame
     *         not yet available.
     */
    public ObjectBlock[] getDetectedObjects()
    {
        final String funcName = "getDetectedObjects";
        ObjectBlock[] objectBlocks = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        synchronized (this)
        {
            objectBlocks = detectedObjects;
            detectedObjects = null;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        return objectBlocks;
    }   //getDetectedObjects

    /**
     * This method processes the data from the read completion handler.
     *
     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
     * @param data specifies the data read.
     * @param length specifies the number of bytes read.
     */
    private void processData(RequestTag requestTag, byte[] data, int length)
    {
        final String funcName = "processData";
        int word;

        if (debugEnabled)
        {
            dbgTrace.traceVerbose(funcName, "tag=%s,data=%s,len=%d", requestTag, Arrays.toString(data), length);
        }

        switch (requestTag)
        {
            case SYNC:
                //
                // If we don't already have an object block allocated, allocate it now.
                //
                if (currBlock == null)
                {
                    currBlock = new ObjectBlock();
                }

                if (length != 2)
                {
                    //
                    // We should never get here. But if we do, probably due to device read failure, we will initiate
                    // another read for SYNC.
                    //
                    asyncReadData(RequestTag.SYNC, 2);
                    if (debugEnabled)
                    {
                        dbgTrace.traceWarn(funcName, "Unexpected data length %d in %s", length, requestTag);
                    }
                }
                else
                {
                    word = getWord(data[0], data[1], msbFirst);
                    if (word == PIXY_START_WORD || word == PIXY_START_WORD_CC)
                    {
                        //
                        // Found a sync word, initiate the read for CHECKSUM.
                        //
                        currBlock.sync = word;
                        asyncReadData(RequestTag.CHECKSUM, 2);
                    }
                    else if (word == PIXY_START_WORDX)
                    {
                        //
                        // We are word misaligned. Realign it by reading one byte and expecting it to be the high
                        // sync byte.
                        //
                        currBlock.sync = PIXY_START_WORD;
                        asyncReadData(RequestTag.ALIGN, 1);
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Word misaligned, realigning...");
                        }
                    }
                    else
                    {
                        //
                        // We don't find the sync word, throw it away and initiate another read for SYNC.
                        //
                        asyncReadData(RequestTag.SYNC, 2);
                        if (debugEnabled)
                        {
                            if (word != 0)
                            {
                                dbgTrace.traceWarn(funcName, "Unexpected word 0x%04x read in %s", word, requestTag);
                            }
                        }
                    }
                }
                break;

            case ALIGN:
                if (length != 1)
                {
                    //
                    // We should never come here. Let's throw an exception to catch this unlikely scenario.
                    //
                    throw new IllegalStateException(String.format("Unexpected data length %d in %s.",
                        length, requestTag));
                }
                else if (data[0] == PIXY_SYNC_HIGH)
                {
                    //
                    // Found the expected upper sync byte, so initiate the read for CHECKSUM.
                    //
                    asyncReadData(RequestTag.CHECKSUM, 2);
                }
                else
                {
                    //
                    // Don't see the expected upper sync byte, let's initiate another read for SYNC assuming we are
                    // now word aligned again.
                    //
                    asyncReadData(RequestTag.SYNC, 2);
                    if (debugEnabled)
                    {
                        dbgTrace.traceWarn(funcName, "Unexpected data byte 0x%02x in %s", data[0], requestTag);
                    }
                }
                break;

            case CHECKSUM:
                if (length != 2)
                {
                    //
                    // We should never come here. Let's throw an exception to catch this unlikely scenario.
                    //
                    throw new IllegalStateException(String.format("Unexpected data length %d in %s.",
                        length, requestTag));
                }
                else
                {
                    word = getWord(data[0], data[1], msbFirst);
                    if (word == PIXY_START_WORD || word == PIXY_START_WORD_CC)
                    {
                        //
                        // We were expecting a checksum but found a sync word. It means that's the end-of-frame.
                        // Save away the sync word for the next frame and initiate the next read for CHECKSUM.
                        //
                        currBlock.sync = word;
                        asyncReadData(RequestTag.CHECKSUM, 2);
                        //
                        // Detected end-of-frame, convert the array list of objects into detected object array.
                        //
                        if (objects.size() > 0)
                        {
                            synchronized (this)
                            {
                                ObjectBlock[] array = new ObjectBlock[objects.size()];
                                detectedObjects = objects.toArray(array);
                                objects.clear();
                                if (debugEnabled)
                                {
                                    for (int i = 0; i < detectedObjects.length; i++)
                                    {
                                        dbgTrace.traceInfo(funcName, "[%02d] %s", i, detectedObjects[i].toString());
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        //
                        // Looks like we have a checksum, save it away and initiate the read for the rest of the
                        // block. If the sync word was PIXY_START_WORD, then it is a 10-byte NORMAL_BLOCK, else it
                        // is a 12-byte COLOR_CODE_BLOCK.
                        //
                        currBlock.checksum = word;
                        if (currBlock.sync == PIXY_START_WORD)
                        {
                            asyncReadData(RequestTag.NORMAL_BLOCK, 10);
                        }
                        else if (currBlock.sync == PIXY_START_WORD_CC)
                        {
                            asyncReadData(RequestTag.COLOR_CODE_BLOCK, 12);
                        }
                        else
                        {
                            //
                            // We should never come here. Let's throw an exception to catch this unlikely scenario.
                            //
                            throw new IllegalStateException(String.format("Unexpected sync word 0x%04x in %s.",
                                currBlock.sync, requestTag));
                        }
                    }
                }
                break;

            case NORMAL_BLOCK:
            case COLOR_CODE_BLOCK:
                if (requestTag == RequestTag.NORMAL_BLOCK && length != 10 ||
                    requestTag == RequestTag.COLOR_CODE_BLOCK && length != 12)
                {
                    //
                    // We should never come here. Let's throw an exception to catch this unlikely scenario.
                    //
                    throw new IllegalStateException(String.format("Unexpected data length %d in %s.",
                        length, requestTag));
                }
                else
                {
                    int index;
                    runningChecksum = 0;
                    //
                    // Save away the signature and accumulate checksum.
                    //
                    index = 0;
                    word = getWord(data[index], data[index + 1], msbFirst);
                    runningChecksum += word;
                    currBlock.signature = word;
                    //
                    // Save away the object center X and accumulate checksum.
                    //
                    index += 2;
                    word = getWord(data[index], data[index + 1], msbFirst);
                    runningChecksum += word;
                    currBlock.centerX = word;
                    //
                    // Save away the object center Y and accumulate checksum.
                    //
                    index += 2;
                    word = getWord(data[index], data[index + 1], msbFirst);
                    runningChecksum += word;
                    currBlock.centerY = word;
                    //
                    // Save away the object width and accumulate checksum.
                    //
                    index += 2;
                    word = getWord(data[index], data[index + 1], msbFirst);
                    runningChecksum += word;
                    currBlock.width = word;
                    //
                    // Save away the object height and accumulate checksum.
                    //
                    index += 2;
                    word = getWord(data[index], data[index + 1], msbFirst);
                    runningChecksum += word;
                    currBlock.height = word;
                    //
                    // If it is a COLOR_CODE_BLOCK, save away the object angle and accumulate checksum.
                    //
                    if (requestTag == RequestTag.COLOR_CODE_BLOCK)
                    {
                        index += 2;
                        word = getWord(data[index], data[index + 1], msbFirst);
                        runningChecksum += word;
                        currBlock.angle = word;
                    }

                    if (runningChecksum == currBlock.checksum)
                    {
                        //
                        // Checksum is correct, add the object block.
                        //
                        objects.add(currBlock);
                        currBlock = null;
                    }
                    else if (debugEnabled)
                    {
                        dbgTrace.traceWarn(funcName, "Incorrect checksum %d (expecting %d).",
                            runningChecksum, currBlock.checksum);
                    }
                    //
                    // Initiate the read for the SYNC word of the next block.
                    //
                    asyncReadData(RequestTag.SYNC, 2);
                }
                break;

            default:
                //
                // We should never come here. Let's throw an exception to catch this unlikely scenario.
                //
                throw new IllegalStateException(String.format("Unexpected request tag %s.", requestTag));
        }
    }   //processData

    /**
     * This method combines the two byte into a 16-bit word according to whether the MSB is first.
     *
     * @param firstByte specifies the first byte.
     * @param secondByte specifies the second byte.
     * @param msbFirst specifies true if first byte is the MSB.
     * @return combined 16-bit word.
     */
    private int getWord(byte firstByte, byte secondByte, boolean msbFirst)
    {
        return msbFirst? TrcUtil.bytesToInt(secondByte, firstByte): TrcUtil.bytesToInt(firstByte, secondByte);
    }   //getWord

    //
    // Implements TrcNotifier.Receiver interface.
    //

    /**
     * This method is called when the read request is completed.
     *
     * @param context specifies the read request.
     */
    @Override
    public void notify(Object context)
    {
        final String funcName = "notify";
        TrcSerialBusDevice.Request request = (TrcSerialBusDevice.Request) context;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK, "request=%s", request);
        }

        if (request.readRequest)
        {
            if (request.readRequest && request.address == -1 &&
                !request.error && !request.canceled && request.buffer != null)
            {
                processData((RequestTag)request.requestCtxt, request.buffer, request.buffer.length);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK);
        }
    }   //notify

}   //class FrcPixyCam
