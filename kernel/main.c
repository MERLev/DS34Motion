/*
 *  DSMotion kernel plugin
 *  Copyright (c) 2017 OperationNT
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:

 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.

 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */
#include <psp2kern/kernel/modulemgr.h>
#include <psp2kern/kernel/threadmgr.h>
#include <psp2kern/kernel/sysmem.h>
#include <psp2kern/kernel/suspend.h>
#include <psp2kern/bt.h>
#include <psp2/motion.h>
#include <taihen.h>

//#include "log.h"
#include <string.h>
#include "../DSMotionLibrary.h"

// Comment this define to have smoother orientation (but some movements will be ignored)
#define EULER_ANGLES

#define abs(val) ((val < 0) ? -val : val)
#define sign(val) ((val > 0) ? 1 : ((val < 0) ? -1 : 0))

static float identityMat[16] = {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};
static SceFQuaternion identityQuat = {0.f, 0.f, 0.f, 1.f};

static unsigned int initTimestamp;
static unsigned int initCounter;

static SceUID mutex_bt_uid = -1;

extern unsigned int ksceKernelGetSystemTimeLow();

#define SONY_VID 0x054C

#define DS3_PID 0x0268

struct ds3_input_report {
	unsigned char report_id;
	unsigned char unk0;

	unsigned char select : 1;
	unsigned char l3     : 1;
	unsigned char r3     : 1;
	unsigned char start  : 1;
	unsigned char up     : 1;
	unsigned char right  : 1;
	unsigned char down   : 1;
	unsigned char left   : 1;

	unsigned char l2       : 1;
	unsigned char r2       : 1;
	unsigned char l1       : 1;
	unsigned char r1       : 1;
	unsigned char triangle : 1;
	unsigned char circle   : 1;
	unsigned char cross    : 1;
	unsigned char square   : 1;

	unsigned char ps       : 1;
	unsigned char not_used : 7;

	unsigned char unk1;

	unsigned char left_x;
	unsigned char left_y;
	unsigned char right_x;
	unsigned char right_y;

	unsigned int unk2;

	unsigned char up_sens;
	unsigned char right_sens;
	unsigned char down_sens;
	unsigned char left_sens;

	unsigned char L2_sens;
	unsigned char R2_sens;
	unsigned char L1_sens;
	unsigned char R1_sens;

	unsigned char triangle_sens;
	unsigned char circle_sens;
	unsigned char cross_sens;
	unsigned char square_sens;

	unsigned short unk3;
	unsigned char unk4;

	unsigned char status;
	unsigned char power_rating;
	unsigned char comm_status;
	unsigned int unk5;
	unsigned int unk6;
	unsigned char unk7;

	unsigned short accel_x;
	unsigned short accel_y;
	unsigned short accel_z;

	union {
		unsigned short gyro_z;
		unsigned short roll;
	};
} __attribute__((packed, aligned(32)));

#define DS4_PID   0x05C4
#define DS4_2_PID 0x09CC

struct ds4_input_report {
	unsigned char report_id;
	unsigned char left_x;
	unsigned char left_y;
	unsigned char right_x;
	unsigned char right_y;

	unsigned char dpad     : 4;
	unsigned char square   : 1;
	unsigned char cross    : 1;
	unsigned char circle   : 1;
	unsigned char triangle : 1;

	unsigned char l1      : 1;
	unsigned char r1      : 1;
	unsigned char l2      : 1;
	unsigned char r2      : 1;
	unsigned char share   : 1;
	unsigned char options : 1;
	unsigned char l3      : 1;
	unsigned char r3      : 1;

	unsigned char ps   : 1;
	unsigned char tpad : 1;
	unsigned char cnt1 : 6;

	unsigned char l_trigger;
	unsigned char r_trigger;

	unsigned char cnt2;
	unsigned char cnt3;

	unsigned char battery;

	signed short accel_x;
	signed short accel_y;
	signed short accel_z;

	union {
		signed short roll;
		signed short gyro_z;
	};
	union {
		signed short yaw;
		signed short gyro_y;
	};
	union {
		signed short pitch;
		signed short gyro_x;
	};

	unsigned char unk1[5];

	unsigned char battery_level : 4;
	unsigned char usb_plugged   : 1;
	unsigned char headphones    : 1;
	unsigned char microphone    : 1;
	unsigned char padding       : 1;

	unsigned char unk2[2];
	unsigned char trackpadpackets;
	unsigned char packetcnt;

	unsigned int finger1_id        : 7;
	unsigned int finger1_activelow : 1;
	unsigned int finger1_x         : 12;
	unsigned int finger1_y         : 12;

	unsigned int finger2_id        : 7;
	unsigned int finger2_activelow : 1;
	unsigned int finger2_x         : 12;
	unsigned int finger2_y         : 12;

} __attribute__((packed, aligned(32)));

struct accelGyroData
{
    signed short accel[3];
    signed short gyro[3];
    unsigned int timestamp;
    unsigned int counter;
};

static int ds3_connected = 0;
static struct ds3_input_report ds3_input;

static int ds4_connected = 0;
static struct ds4_input_report ds4_input;

static unsigned char* recv_buff = NULL;
static unsigned int ds_mac0 = 0;
static unsigned int ds_mac1 = 0;

#define NB_DATA 64

static struct accelGyroData previousData[NB_DATA];
static int currentData = NB_DATA-1;
static int globalCounter = 0;

static float fastsqrt(float val)
{
    union
    {
        int tmp;
        float f;
    } u;
    u.f = val;
    u.tmp -= 1 << 23; /* Subtract 2^m. */
    u.tmp >>= 1; /* Divide by 2. */
    u.tmp += 1 << 29; /* Add ((b + 1) / 2) * 2^m. */
    return u.f;
}

#define M_PI 3.14159265359f

static float sine(float x)
{
    static float B = 4.f/M_PI;
    static float C = -4.f/(M_PI*M_PI);

    float y = B * x + C * x * abs(x);

    //  const float Q = 0.775;
    float P = 0.225f;
    y = P * (y * abs(y) - y) + y;   // Q * y + P * y * abs(y)
    return y;
}

static float cosine(float x)
{
    return sine(x + (M_PI / 2.f));
}

#ifdef EULER_ANGLES

float atan2_approx(float y, float x)
{
    static float ONEQTR_PI = M_PI / 4.0;
	static float THRQTR_PI = 3.0 * M_PI / 4.0;
	float r, angle;
	float abs_y = abs(y) + 1e-10f;
	if ( x < 0.0f )
	{
		r = (x + abs_y) / (abs_y - x);
		angle = THRQTR_PI;
	}
	else
	{
		r = (x - abs_y) / (x + abs_y);
		angle = ONEQTR_PI;
	}
	angle += (0.1963f * r * r - 0.9817f) * r;
	if ( y < 0.0f )
		return -angle;

    return angle;
}

static void eulerToQuaternion(SceFQuaternion* quat, float x, float y, float z)
{
	float cy = cosine(z * 0.5f);
	float sy = sine(z * 0.5f);
	float cr = cosine(y * 0.5f);
	float sr = sine(y * 0.5f);
	float cp = cosine(x * 0.5f);
	float sp = sine(x * 0.5f);

	quat->w = cy * cr * cp + sy * sr * sp;
	quat->x = cy * sr * cp - sy * cr * sp;
	quat->y = cy * cr * sp + sy * sr * cp;
	quat->z = sy * cr * cp - cy * sr * sp;
}

#else

static float arccosine(float x)
{
    float a=1.43f+0.59f*x;
    a=(a+(2.f+2.f*x)/a)/2.f;
    float b=1.65f-1.41f*x;
    b=(b+(2.f-2.f*x)/b)/2.f;
    float c=0.88f-0.77f*x;
    c=(c+(2.f-a)/c)/2.f;
    return 8.f/3.f*c-b/3.f;
}

#endif

/*static void quaternionProduct(SceFQuaternion* res, SceFQuaternion* q1, SceFQuaternion* q2)
{
    res->w = q1->w*q2->w - q1->x*q2->x - q1->y*q2->y - q1->z*q2->z;
    res->x = q1->w*q2->x + q1->x*q2->w - q1->y*q2->z + q1->z*q2->y;
    res->y = q1->w*q2->y + q1->x*q2->z + q1->y*q2->w - q1->z*q2->x;
    res->z = q1->w*q2->z - q1->x*q2->y + q1->y*q2->x + q1->z*q2->w;
}*/

static int computeQuaternionFromAccel(SceFQuaternion* oRes, SceFVector3* iAccel)
{
    float accelNorm = fastsqrt(iAccel->x*iAccel->x + iAccel->y*iAccel->y + iAccel->z*iAccel->z);
    if (accelNorm < 0.001f)
        return 0;

    SceFVector3 normAccel = { iAccel->x / accelNorm , iAccel->y / accelNorm , iAccel->z / accelNorm };
    
#ifdef EULER_ANGLES
    float pitch = atan2_approx(normAccel.z, -normAccel.y);
    float roll = atan2_approx(-normAccel.x, -normAccel.z*sign(-pitch));

    eulerToQuaternion(oRes, 0.f, pitch, roll);
#else
    static SceFVector3 initDir = {0.f, -1.f, 0.f};
    
    oRes->x = initDir.z*normAccel.y - initDir.y*normAccel.z;
    oRes->y = initDir.x*normAccel.z - initDir.z*normAccel.x;
    oRes->z = initDir.y*normAccel.x - initDir.x*normAccel.y;
    
    float angle = arccosine(initDir.x*normAccel.x + initDir.y*normAccel.y + initDir.z*normAccel.z);
    float half_sin = sine(0.5f * angle);
    float half_cos = cosine(0.5f * angle);
    oRes->w = half_cos;
    
    float crossNorm = fastsqrt(oRes->x*oRes->x + oRes->y*oRes->y + oRes->z*oRes->z);
    oRes->x *= half_sin / crossNorm;
    oRes->y *= half_sin / crossNorm;
    oRes->z *= half_sin / crossNorm;
#endif
    
    return 1;
}

unsigned int dsGetCurrentTimestamp()
{
    return ksceKernelGetSystemTimeLow();
}

unsigned int dsGetCurrentCounter()
{
    return globalCounter;
}

unsigned int dsGetSampledAccelGyro(unsigned int iSamplingTimeMS, signed short oAccel[3], signed short oGyro[3])
{
    if (!ds3_connected && !ds4_connected)
        return 0;

    int initIndex = currentData;
    struct accelGyroData* data = &previousData[initIndex];

    int accel_sum[3] = {data->accel[0], data->accel[1], data->accel[2]};
    int gyro_sum[3] = {data->gyro[0], data->gyro[1], data->gyro[2]};

    unsigned int initTime = data->timestamp;
    unsigned int samplingTimeNano = 1000 * iSamplingTimeMS;

    signed short index;
    for (index = 1; index < NB_DATA; index++)
    {
        int dataIndex = (initIndex-index+NB_DATA)%NB_DATA;
        data = &previousData[dataIndex];
        
        if (0 == data->counter || initTime-data->timestamp > samplingTimeNano)
            break;
        
        accel_sum[0] += data->accel[0];
        accel_sum[1] += data->accel[1];
        accel_sum[2] += data->accel[2];

        gyro_sum[0] += data->gyro[0];
        gyro_sum[1] += data->gyro[1];
        gyro_sum[2] += data->gyro[2];
    }
    
    signed short accel[3] = {accel_sum[0] / index, accel_sum[1] / index, accel_sum[2] / index};
    signed short gyro[3] = {gyro_sum[0] / index, gyro_sum[1] / index, gyro_sum[2] / index};

	memcpy(oAccel, (const void *)accel, 3*sizeof(signed short));
    memcpy(oGyro, (const void *)gyro, 3*sizeof(signed short));

    return index;
}

int dsGetInstantAccelGyro(unsigned int iIndex, struct accelGyroData* oData)
{
    if (!ds3_connected && !ds4_connected)
        return -1;

    int curIndex = (currentData-(iIndex%NB_DATA)+NB_DATA)%NB_DATA;
    memcpy(oData, (const void *)&previousData[curIndex], sizeof(struct accelGyroData));

    return 0;
}


int dsMotionGetState(SceMotionState *ms)
{
    SceMotionState motionState;
	memset(&motionState, 0, sizeof(motionState));
    signed short accel[3];
    signed short gyro[3];

    if (dsGetSampledAccelGyro(100, accel, gyro) > 0)
    {
        motionState.hostTimestamp = ksceKernelGetSystemTimeWide();

        motionState.acceleration.x = -(float)accel[2] / 0x2000;
        motionState.acceleration.y = (float)accel[0] / 0x2000;
        motionState.acceleration.z = -(float)accel[1] / 0x2000;

        // 2607.6 = 0x2000 / PI
        motionState.angularVelocity.x = (float)gyro[0] / 2607.6f;  // Pitch
        motionState.angularVelocity.y = -(float)gyro[2] / 2607.6f; // Roll
        motionState.angularVelocity.z = (float)gyro[1] / 2607.6f;  // Yaw
        
        int maxComp = (abs(accel[1]) > abs(accel[0])) ? 1 : 0;
        maxComp = (abs(accel[2]) > abs(accel[maxComp])) ? 2 : maxComp;

        motionState.basicOrientation.x = (2 == maxComp) ? sign(accel[2]) : 0.f;
        motionState.basicOrientation.y = (0 == maxComp) ? -sign(accel[0]) : 0.f;
        motionState.basicOrientation.z = (1 == maxComp) ? sign(accel[1]) : 0.f;

        if (computeQuaternionFromAccel(&motionState.deviceQuat, &motionState.acceleration))
        {
            float sqx = motionState.deviceQuat.x*motionState.deviceQuat.x;
            float sqy = motionState.deviceQuat.y*motionState.deviceQuat.y;
            float sqz = motionState.deviceQuat.z*motionState.deviceQuat.z;
            float sqw = motionState.deviceQuat.w*motionState.deviceQuat.w;
            float invs = 1.f / (sqx + sqy + sqz + sqw);
            
            float* rotMat = (float*)&motionState.rotationMatrix;
            rotMat[0]  = ( sqx - sqy - sqz + sqw) * invs;
            rotMat[5]  = (-sqx + sqy - sqz + sqw) * invs;
            rotMat[10] = (-sqx - sqy + sqz + sqw) * invs;
            
            float tmp1 = motionState.deviceQuat.x*motionState.deviceQuat.y;
            float tmp2 = motionState.deviceQuat.z*motionState.deviceQuat.w;
            rotMat[4] = 2.f * (tmp1 + tmp2) * invs;
            rotMat[1] = 2.f * (tmp1 - tmp2) * invs;
            
            tmp1 = motionState.deviceQuat.x*motionState.deviceQuat.z;
            tmp2 = motionState.deviceQuat.y*motionState.deviceQuat.w;
            rotMat[8] = 2.f * (tmp1 - tmp2) * invs ;
            rotMat[2] = 2.f * (tmp1 + tmp2) * invs ;
            
            tmp1 = motionState.deviceQuat.y*motionState.deviceQuat.z;
            tmp2 = motionState.deviceQuat.x*motionState.deviceQuat.w;
            rotMat[9] = 2.f * (tmp1 + tmp2) * invs;
            rotMat[6] = 2.f * (tmp1 - tmp2) * invs;
            
            rotMat[3] = rotMat[7] = rotMat[11] = rotMat[12] = rotMat[13] = rotMat[14] = 0.f;
            rotMat[15] = 1.f;
        }
        else
        {
            memcpy(&motionState.deviceQuat, &identityQuat, sizeof(identityQuat));
            memcpy(&motionState.rotationMatrix, identityMat, sizeof(identityMat));
        }
        
        memcpy(&motionState.nedMatrix, identityMat, sizeof(identityMat));
		ksceKernelMemcpyKernelToUser((uintptr_t)ms, (const void *)&motionState, sizeof(motionState));

        return 0;
    }
    return -1;
}

int dsMotionGetSensorState(SceMotionSensorState *sensorState, int numRecords){
    struct accelGyroData data;
    for (int i = 0 ; i < numRecords ; i++)
    {
        if (dsGetInstantAccelGyro(numRecords-1-i, &data) >= 0)
        {
            // SceMotionSensorState* curState = &sensorState[i];
            SceMotionSensorState curState;

            curState.accelerometer.x = -(float)data.accel[2] / 0x2000;
            curState.accelerometer.y = (float)data.accel[0] / 0x2000;
            curState.accelerometer.z = -(float)data.accel[1] / 0x2000;

            // 2608.6 = 0x2000 / PI
            curState.gyro.x = (float)data.gyro[0] / 2607.6f;
            curState.gyro.y = -(float)data.gyro[2] / 2608.6f;
            curState.gyro.z = (float)data.gyro[1] / 2608.6f;

            curState.timestamp = data.timestamp - initTimestamp;
            curState.counter = data.counter - initCounter;
			ksceKernelMemcpyKernelToUser((uintptr_t)&sensorState[i], (const void *)&curState, sizeof(curState));
        }
    }
    return 0;
}

int dsMotionStartSampling(){
    initTimestamp = dsGetCurrentTimestamp();
    initCounter = dsGetCurrentCounter();
	return 0;
}

static inline void ds3_input_reset(void)
{
	memset(&ds3_input, 0, sizeof(ds3_input));
}

static inline void ds4_input_reset(void)
{
	memset(&ds4_input, 0, sizeof(ds4_input));
}

static int is_ds3(const unsigned short vid_pid[2])
{
	return vid_pid[0] == SONY_VID && vid_pid[1] == DS3_PID;
}

static int is_ds4(const unsigned short vid_pid[2])
{
	return (vid_pid[0] == SONY_VID) && ((vid_pid[1] == DS4_PID) || (vid_pid[1] == DS4_2_PID));
}

#define DECL_FUNC_HOOK(name, ...) \
	static tai_hook_ref_t name##_ref; \
	static SceUID name##_hook_uid = -1; \
	static int name##_hook_func(__VA_ARGS__)

DECL_FUNC_HOOK(SceBt_ksceBtReadEvent, SceBtEvent *events, int num_events)
{
	int ret = TAI_CONTINUE(int, SceBt_ksceBtReadEvent_ref, events, num_events);

	if (ret >= 0)
    {
        for (int i = 0 ; i < num_events ; i++)
        {
            SceBtEvent* event = &events[i];
            //LOG("Connection event %d with %d %d\n", event->id, event->mac0, event->mac1);

            if (!ds3_connected && !ds4_connected && 0x05 == event->id)
            {
                unsigned short vid_pid[2];
                unsigned int result1 = ksceBtGetVidPid(event->mac0, event->mac1, vid_pid);
                //LOG("Vendor ID %d ; Product ID %d\n", vid_pid[0], vid_pid[1]);
                //log_flush();

                int connected = 0;
                if (is_ds4(vid_pid))
                {
                    ds4_input_reset();
                    ds4_connected = 1;
                    connected = 1;
                }
                else
                {
                    char name[0x79];
                    unsigned int result2 = ksceBtGetDeviceName(event->mac0, event->mac1, name);
                    if (is_ds3(vid_pid)|| (result1 == 0x802F5001 && result2 == 0x802F0C01))
                    {
                        ds3_input_reset();
                        ds3_connected = 1;
                        connected = 1;
                    }
                }
                
                if (connected)
                {
                    ds_mac0 = event->mac0;
                    ds_mac1 = event->mac1;
                    globalCounter = 0;
                }
            }
            else if ((ds3_connected || ds4_connected) && event->mac0 == ds_mac0 && event->mac1 == ds_mac1)
            {
                ksceKernelLockMutex(mutex_bt_uid, 1, NULL);

                if (0x06 == event->id)
                {
                    ds3_connected = 0;
                    ds4_connected = 0;
                }
                else if (NULL != recv_buff)
                {
                    if (0x0A == event->id)
                    {
                        if ((ds4_connected && 0x11 == recv_buff[0]) || (ds3_connected && 0x01 == recv_buff[0]))
                        {
                            int newData = (currentData+1)%NB_DATA;
                            struct accelGyroData* data = &previousData[newData];

                            if (ds4_connected)
                            {
                                memcpy(&ds4_input, recv_buff, sizeof(ds4_input));

                                // Data from gyroscope and accelerometer seem inverted on DS4
                                data->accel[0] = ds4_input.gyro_x;
                                data->accel[1] = ds4_input.gyro_y;
                                data->accel[2] = ds4_input.gyro_z;

                                data->gyro[0] = ds4_input.accel_x;
                                data->gyro[1] = ds4_input.accel_y;
                                data->gyro[2] = ds4_input.accel_z;
                            }
                            else // if (ds3_connected)
                            {
                                memcpy(&ds3_input, recv_buff, sizeof(ds3_input));

                                // DS3 matching with DS4
                                data->accel[0] = -((signed short)ds3_input.accel_y)/4;
                                data->accel[1] = -((signed short)ds3_input.accel_z)/4;
                                data->accel[2] = ((signed short)ds3_input.accel_x)/4;

                                data->gyro[0] = 0;
                                data->gyro[1] = ((signed short)ds3_input.gyro_z+0x15FF)/10;
                                data->gyro[2] = 0;
                            }

                            data->timestamp = ksceKernelGetSystemTimeLow();
                            data->counter = (++globalCounter);
                            currentData = newData;
                        }
                        recv_buff = NULL;
                    }
                    else if (0x0B == event->id || 0x0C == event->id)
                        recv_buff = NULL;
                }
                
                ksceKernelUnlockMutex(mutex_bt_uid, 1);
            }
        }
	}

	return ret;
}

DECL_FUNC_HOOK(SceBt_ksceBtHidTransfer, unsigned int mac0, unsigned int mac1, SceBtHidRequest *request)
{
	int ret = TAI_CONTINUE(int, SceBt_ksceBtHidTransfer_ref, mac0, mac1, request);

    if (ret >= 0 && (ds3_connected || ds4_connected) && mac0 == ds_mac0 && mac1 == ds_mac1)
    {
        ksceKernelLockMutex(mutex_bt_uid, 1, NULL);

        if (NULL != request && NULL != request->buffer && request->length >= (ds4_connected?sizeof(ds4_input):sizeof(ds3_input)))
            recv_buff = (unsigned char*)request->buffer;
        else
            recv_buff = NULL;

        ksceKernelUnlockMutex(mutex_bt_uid, 1);
    }
    
    return ret;
}

void _start() __attribute__ ((weak, alias ("module_start")));

#define BIND_FUNC_EXPORT_HOOK(name, pid, module, lib_nid, func_nid) \
	name##_hook_uid = taiHookFunctionExportForKernel((pid), \
		&name##_ref, (module), (lib_nid), (func_nid), name##_hook_func)

int module_start(SceSize argc, const void *args)
{
	int ret;
	tai_module_info_t SceBt_modinfo;

	//log_reset();
	//LOG("dsmotion kernel by OperationNT\n");

	SceBt_modinfo.size = sizeof(SceBt_modinfo);
	ret = taiGetModuleInfoForKernel(KERNEL_PID, "SceBt", &SceBt_modinfo);
	if (ret < 0) {
		//LOG("Error finding SceBt module\n");
		goto error_find_scebt;
	}

    // mutex_ds_motion_state_uid = ksceKernelCreateMutex("mutex_ds_motion_state", 0, 0, NULL);
    mutex_bt_uid = ksceKernelCreateMutex("mutex_bt", 0, 0, NULL);

	/* SceBt hooks */
	BIND_FUNC_EXPORT_HOOK(SceBt_ksceBtReadEvent, KERNEL_PID, "SceBt", TAI_ANY_LIBRARY, 0x5ABB9A9D);
    //LOG("ksceBtReadEvent hook result: %x\n", SceBt_ksceBtReadEvent_hook_uid);

	BIND_FUNC_EXPORT_HOOK(SceBt_ksceBtHidTransfer, KERNEL_PID, "SceBt", TAI_ANY_LIBRARY, 0xF9DCEC77);
    //LOG("ksceBtHidTransfer hook result: %x\n", SceBt_ksceBtHidTransfer_hook_uid);
    
	//LOG("module_start finished successfully!\n");
    //log_flush();
    
    memset(previousData, 0, sizeof(previousData));

	return SCE_KERNEL_START_SUCCESS;

error_find_scebt:
	return SCE_KERNEL_START_FAILED;
}

#define UNBIND_FUNC_HOOK(name) \
	do { \
		if (name##_hook_uid > 0) { \
			taiHookReleaseForKernel(name##_hook_uid, name##_ref); \
		} \
	} while(0)

int module_stop(SceSize argc, const void *args)
{
	UNBIND_FUNC_HOOK(SceBt_ksceBtReadEvent);
    UNBIND_FUNC_HOOK(SceBt_ksceBtHidTransfer);

	//log_flush();
    if (mutex_bt_uid >= 0){
        ksceKernelDeleteMutex(mutex_bt_uid);
    }

	return SCE_KERNEL_STOP_SUCCESS;
}
