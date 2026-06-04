#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>//select函数需要的头文件，提供timeval结构体和相关函数，用于设置select函数的超时时间。
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <linux/videodev2.h>//V4L2的核心头文件，定义了与视频设备交互所需的结构体、常量和函数原型。
#include <libv4l2.h>
#include <stdint.h>
#include <inttypes.h>
#include <pthread.h>
#include "v4l2camera.h"

/*

数据流：
摄像头硬件->驱动->内核buffer->mmap ->用户空间
核心操作：
1、VIDIOC_REQBUFS 申请buffer
2、VIDIOC_QUERYBUF 查询buffer
3、mmap映射内存
4、VIDIOC_QBUF 入队
5、VIDIOC_STREAMON开始采集
6、VIDIOC_DQBUF取出一帧
7、VIDIOC_QBUF放回buffer
*/
//宏定义；申请四个缓冲区域
#define VIDIOC_REQBUFS_COUNT 4
// 全局状态结构体（整个模块的核心数据中心）
static struct v4l2cameraPara_s
{
    int is_init;   //标志是否已初始化
    int fd;//摄像头文件描述符，open的返回句柄，硬件直接访问内存数据必须的
    char device[20];//设备路径字符串
    int width;//图像宽度
    int height;//图像高度
    int format;//像素格式，如V4L2_PIX_FMT_NV12
    int fps;//帧率

    //camera_buffer结构体定义在v4l2camera.h中。
    struct camera_buffer* buffers; // **修改**，指向struct camera_buffer数组的指针，用于存储每个缓冲区的信息（包括内存映射地址、长度、文件描述符等）
    unsigned int n_buffers;//buffer数量
    unsigned int num_planes;//plane数量
    pthread_t camera_pid;//捕获线程id
    int camera_pid_on;//线程运行标志
    v4l2camera_callback_t callback;//用户设置的回调函数，当捕获到一帧图像时调用（用于把图像传出去）
} g_para =//使用C99指定初始化器将所有字段置为默认值。（全局状态变量，初始化为默认值，未初始化状态，文件描述符无效，设备路径空，分辨率和帧率为0，格式为NV12，缓冲区指针为NULL，线程未启动，回调函数为NULL）
{
    .is_init = 0,
    .fd = -1,
    .device = {0},
    .width = 0,
    .height = 0,
    .format = V4L2_PIX_FMT_NV12,
    //.fps = 30,
    .fps = 30,
    .buffers = NULL,
    .n_buffers = 0,
    .num_planes = 0,
    .camera_pid_on = 0,
    .callback = NULL,
};
//ioctl:和驱动通信的唯一接口
//request：命令（如VIDIOC_DQBUF）
//函数定义；封装ioctl系统调用，处理被信号中断的情况（EINTR），确保调用的可靠性。
//返回值：ioctl的返回值，如果成功则为0，失败则为-1，并设置errno。
//参数：fd - 文件描述符，request - ioctl请求代码，argp - 指向请求参数的指针。
static int xioctl(int fd, int request, void* argp)
{
    int r;//r
    do r = ioctl(fd, request, argp);//如果ioctl返回-1且errno是EINTR，继续循环，否则返回结果
    while (-1 == r && EINTR == errno);//r=-1且errno=EINTR表示ioctl调用被信号中断，继续调用ioctl直到成功或遇到其他错误，确保ioctl调用的可靠性。
    return r;//返回ioctl调用的结果
}
//核心线程函数（摄像头采集循环）
//函数定义：摄像头捕获线程函数，负责从摄像头设备读取图像数据并调用用户回调函数处理图像数据。
void* CameraLoopProcess(void* args)//args是线程函数的参数，这里未使用，可以传递一些上下文信息
{
    struct sched_param param;//设置线程调度参数，提升线程优先级，以确保摄像头数据的及时处理
    param.sched_priority = 50; // 1-99，越大越高
    //设置线程为实时调度策略SCHED_FIFO，优先级为50，这需要root权限或CAP_SYS_NICE能力，失败时打印错误信息但继续执行线程（可能会导致性能下降），不影响线程的正常运行。
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        perror("pthread_setschedparam failed"); // 需要 root 权限或 CAP_SYS_NICE
    }
    //
    int ret = -1;//定义一个变量ret用于存储函数调用的返回值，初始值为-1表示错误状态
    //定义并初始化一个v4l2_buffer结构体buf,用于存储从摄像头设备读取的图像数据的相关信息。
    struct v4l2_buffer buf = {0};//此变量用于与摄像头设备进行交互，存储缓冲区的索引、类型、内存方式、平面信息等，初始化为0确保所有字段有默认值，避免未初始化的使用导致错误。
    struct v4l2_plane planes[VIDEO_MAX_PLANES] = {0};//定义一个v4l2_plane数组planes,用于存储每个平面的地址和长度信息，VIDEO_MAX_PLANES是V4L2定义的最大平面数量常量，初始化为0.
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;//表示这是一个多平面视频捕获缓冲区，多平面格式允许每个缓冲区包含多个平面（如YUV格式的Y、U、V分量），每个平面可以有不同的地址和长度。
    buf.memory = V4L2_MEMORY_MMAP;//表示缓冲区使用mmap内存映射方式访问，内存映射允许用户空间直接访问内核空间的缓冲区数据，避免了数据拷贝，提高了性能。
    buf.m.planes = planes;//此变量指向一个v4l2_plane数组，用于存储每个平面的地址和长度信息，摄像头设备会通过这个数组返回每个平面的物理地址和长度，用户空间可以根据这些信息访问图像数据。
    buf.length = g_para.num_planes;//表示平面数量，告诉摄像头设备我们准备了多少个平面来接收图像数据，设备会根据这个值返回相应数量的平面信息。


    //设置select函数的超时时间和文件描述符集合，select函数用于等待摄像头设备有数据可读（使用select监视设备文件描述符的可读事件），设置超时时间为2秒，如果超过这个时间没有数据可读，select函数会返回0表示超时，我们可以在日志中记录这个事件并继续等待数据，出错则退出。在日志中记录的目的是为了监控摄像头数据的流动情况，及时发现可能的性能问题或设备异常。
    struct timeval tv = {0};//定义并初始化一个timeval结构体tv，用于设置select函数的超时时间，tv_sec表示秒，tv_usec表示微秒，初始化为0表示默认没有超时，后续会在循环中设置为2秒，以确保摄像头数据的及时处理，如果超过这个时间并没有数据可读，select函数会返回0则继续循环，表示超时，我们可以在日志中记录这个事件并继续等待数据，出错则退出。在日志中记录的目的是为了监控摄像头数据的流动情况，及时发现可能的性能问题或设备异常。
    fd_set fds;//定义一个fd_set类型的变量fds，用于存储需要监视的文件描述符集合，select函数会检查这些文件描述符是否有事件发生（如可读、可写等），在这里我们将摄像头设备的文件描述符添加到这个集合中，以便等待摄像头数据的到来。

    while(g_para.camera_pid_on)//主循环（不断采集视频帧），当camera_pid_on为真时持续运行，负责从摄像头设备读取图像数据并调用用户回调函数处理图像数据。
    {
        tv.tv_sec = 2;//设置超时时间2s
        tv.tv_usec = 0;//设置超时时间0us
        FD_ZERO(&fds);//清空文件描述符集合，确保之前的监视状态被重置，准备添加新的文件描述符进行监视。
        FD_SET(g_para.fd, &fds);//将摄像头设备的文件描述符添加到监视集合中，表示我们希望等待这个文件描述符上的事件（如数据可读）发生，以便及时处理摄像头数据。
        //阻塞等待摄像头有数据可读
        //ret是select函数的返回值，表示有多少个文件描述符上发生了事件（如可读），如果返回-1表示出错，如果返回0表示超时，如果返回正数表示有事件发生，我们可以根据这个返回值来判断是否需要处理摄像头数据，或者记录超时事件，或者处理错误情况。
        ret = select(g_para.fd + 1, &fds, NULL, NULL, &tv);//调用select函数等待摄像头设备有数据可读，参数说明：第一个参数是监视的文件描述符数量（通常是最大文件描述符加1），第二个参数是指向可读事件集合的指针，第三个和第四个参数分别是指向可写事件集合和异常事件集合的指针（这里我们不监视这两种事件，所以传NULL），最后一个参数是指向timeval结构体的指针，用于设置超时时间，如果超过这个时间没有数据可读，select函数会返回0表示超时，我们可以在日志中记录这个事件并继续等待数据，出错则退出。在日志中记录的目的是为了监控摄像头数据的流动情况，及时发现可能的性能问题或设备异常。
        if (-1 == ret)
        {   //如果select函数返回-1表示出错，进一步检查错误类型，如果是EINTR表示被信号中断，可以继续等待数据，否则打印错误信息并退出循环。
            if (EINTR == errno)//select信号：如果select函数被信号中断（errno为EINTR），继续等待数据，否则打印错误信息并退出循环。errno是全局变量，表示最近一次系统调用的错误代码，EINTR表示系统调用被信号中断，这种情况通常是暂时的，可以继续等待数据，因此我们使用continue语句跳过当前循环的剩余部分，重新进入下一次循环等待数据到来。其他错误则可能是严重的问题，我们需要记录错误信息并退出循环，以避免无限循环或资源泄漏。
                continue;//如果select被信号中断，继续等待
            fprintf(stderr, "CameraLoopProcess select error %d, %s\n", errno, strerror(errno));//打印select函数的错误信息，errno是错误代码，strerror(errno)返回对应的错误描述字符串，这有助于调试和监控摄像头数据的流动情况，及时发现可能的性能问题或设备异常。
            break;//其他错误，退出循环
        }
        //如果select函数返回0表示超时，打印日志提示摄像头数据流动异常（如设备未响应或性能问题），并继续等待数据到来，这有助于监控摄像头数据的流动情况，及时发现可能的性能问题或设备异常。
        if(0 == ret)
        {
            printf("CameraLoopProcess select timeout!\n");
            continue;
        }
        //如果select函数返回正数表示有数据可读，继续执行后续操作，从摄像头设备读取一帧图像数据，使用VIDIOC_DQBUF命令从内核缓冲区中取出一个已填充的缓冲区，如果没有可用的缓冲区或者发生错误，处理相应的情况(如EAGAIN表示没有数据可读)，成功取出一帧图像数据后，调用用户设置的回调函数处理图像数据，如果回调函数不为NULL，则将图像数据通过回调函数传递给用户进行处理（如显示、编码等），如果回调函数为NULL，则打印日志提示回调函数未设置。
        //从摄像头设备读取一帧图像数据，使用VIDIOC_DQBUF命令从内核缓冲区中取出一个已填充的缓冲区，如果没有可用的缓冲区或者发生错误，处理相应的情况(如EAGAIN表示没有数据可读)
        if (-1 == xioctl(g_para.fd, VIDIOC_DQBUF, &buf))//调用封装的xioctl函数执行VIDIOC_DQBUF命令，从内核缓冲区中取出一个已填充的缓冲区，参数说明：第一个参数是摄像头设备的文件描述符，第二个参数是VIDIOC_DQBUF命令，第三个参数是指向v4l2_buffer结构体的指针，用于存储取出的缓冲区信息，如果没有可用的缓冲区或者发生错误，处理相应的情况(如EAGAIN表示没有数据可读)，成功取出一帧图像数据后，调用用户设置的回调函数处理图像数据，如果回调函数不为NULL，则将图像数据通过回调函数传递给用户进行处理（如显示、编码等），如果回调函数为NULL，则打印日志提示回调函数未设置。
        {
            //发生EAGAIN错误表示没有数据可读，并非阻塞模式下的正常情况，我们可以继续等待数据到来，因此使用continue语句跳过当前循环的剩余部分，重新进入下一次循环等待数据到来。这有助于监控摄像头数据的流动情况，及时发现可能的性能问题或设备异常。EIO错误可能为硬件错误、DMA错误等，退出本次循环继续下一帧数据的处理，避免一有错就退出整个线程，保持线程的持续运行以处理后续数据。
            if (errno == EAGAIN || errno == EIO) {
                continue;//如果没有可用的缓冲区或者发生错误，处理相应的情况(如EAGAIN表示没有数据可读)，继续等待数据到来，这有助于监控摄像头数据的流动情况，及时发现可能的性能问题或设备异常。
            } else {
                fprintf(stderr, "VIDIOC_DQBUF error %d, %s\n", errno, strerror(errno));
                break;//其他典型的不可修复的致命错误，退出循环
            }
        }
        //成功取出一帧图像数据后，调用用户设置的回调函数处理图像数据，如果回调函数不为NULL，则将图像数据通过回调函数传递给用户进行处理（如显示、编码等），如果回调函数为NULL，则打印日志提示回调函数未设置。
        assert(buf.index < g_para.n_buffers);//断言取出的缓冲区索引在有效范围内，确保我们访问的缓冲区信息是合法的。buf.index是从内核缓冲区中取出的缓冲区索引，g_para.n_buffers是我们申请的缓冲区数量，如果索引超出范围，说明有严重的逻辑错误，我们使用assert来捕捉这个问题，避免访问非法内存。
        if (g_para.callback)//如果用户设置了回调函数，调用回调函数处理数据，
            //参数说明：第一个参数是指向camera_buffer结构体的指针，包含了当前缓冲区的平面信息，第二个和第三个参数是图像的宽度和高度，第四个参数是通道号，这里暂时固定为1，表示主通道，如果有多通道的需求，可以根据实际情况进行修改和扩展。
            g_para.callback(&g_para.buffers[buf.index], g_para.width, g_para.height, 1);
        else
            printf("v4l2camera module callback is null\n");//如果回调函数未设置，打印日志提示回调函数未设置。

        if (-1 == xioctl(g_para.fd, VIDIOC_QBUF, &buf))//如果成功处理完图像数据后，使用VIDIOC_QBUF命令将缓冲区重新入队，以便摄像头设备可以继续使用这个缓冲区来存储新的图像数据，如果入队失败，打印错误信息并退出循环。
        {
            fprintf(stderr, "VIDIOC_QBUF error %d, %s\n", errno, strerror(errno));
            break;
        }
    }
    printf("CameraLoopProcess thread exiting.\n");
    pthread_exit(0);
}

//
static int V4l2CameraInit(const char* dev_name, unsigned int width, unsigned int height, unsigned int fps, unsigned int format)
{
    struct v4l2_capability cap = {0};
    struct v4l2_format fmt = {0};
    struct v4l2_requestbuffers req = {0};
    enum v4l2_buf_type type;
    unsigned int i;

    if(g_para.is_init)
    {
        printf("g_V4l2Camera already init,ignore it.\n");
        return VC_ERR_ALREADLY_INIT;
    }

    strncpy(g_para.device, dev_name, sizeof(g_para.device));
    g_para.width = width;
    g_para.height = height;
    g_para.format = format;
    g_para.fps = fps;

    // printf("Init Camera device:[%s] res:[%d x %d] fps:[%d]\n", dev_name, width, height, fps);
    g_para.fd = open(dev_name, O_RDWR | O_NONBLOCK, 0);//
    if (-1 == g_para.fd)
    {
        fprintf(stderr, "Cannot open '%s': %d, %s\n", dev_name, errno, strerror(errno));
        return VC_ERR_V4L2_OPEN;
    }

    if (-1 == xioctl(g_para.fd, VIDIOC_QUERYCAP, &cap))
    {
        fprintf(stderr, "VIDIOC_QUERYCAP error %d, %s\n", errno, strerror(errno));
        return VC_ERR_V4L2_IOCTL;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE))
    {
        fprintf(stderr, "%s is no video capture device (MPLANE)\n", dev_name);
        return VC_ERR_V4L2_NOCAPDEV;
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        fprintf(stderr, "%s does not support streaming i/o\n", dev_name);
        return VC_ERR_V4L2_NOSTREAMING;
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;//
    fmt.type = type;//
    fmt.fmt.pix_mp.width = g_para.width;
    fmt.fmt.pix_mp.height = g_para.height;
    fmt.fmt.pix_mp.pixelformat = g_para.format;
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    
    
    
      // ===== 添加打印：输出 fmt 结构体内容 =====
    printf("===== VIDIOC_S_FMT DEBUG =====\n");
    printf("fmt.type = %d (MPLANE=%d, CAPTURE=%d)\n", 
           fmt.type, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_BUF_TYPE_VIDEO_CAPTURE);
    printf("fmt.fmt.pix_mp.width  = %d\n", fmt.fmt.pix_mp.width);
    printf("fmt.fmt.pix_mp.height = %d\n", fmt.fmt.pix_mp.height);
    printf("fmt.fmt.pix_mp.pixelformat = 0x%08x (%c%c%c%c)\n", 
           fmt.fmt.pix_mp.pixelformat,
           fmt.fmt.pix_mp.pixelformat & 0xFF,
           (fmt.fmt.pix_mp.pixelformat >> 8) & 0xFF,
           (fmt.fmt.pix_mp.pixelformat >> 16) & 0xFF,
           (fmt.fmt.pix_mp.pixelformat >> 24) & 0xFF);
    printf("fmt.fmt.pix_mp.field = %d\n", fmt.fmt.pix_mp.field);
    printf("fmt.fmt.pix_mp.num_planes = %d\n", fmt.fmt.pix_mp.num_planes);
    printf("g_para.format = 0x%08x\n", g_para.format);
    printf("==============================\n");
    // ===== 打印结束 =====

    if (-1 == xioctl(g_para.fd, VIDIOC_S_FMT, &fmt))
    {
        fprintf(stderr, "VIDIOC_S_FMT error %d, %s\n", errno, strerror(errno));
        return VC_ERR_V4L2_IOCTL;
    }

    g_para.width = fmt.fmt.pix_mp.width;//更新实际设置的宽度，某些设备可能会调整到最接近的支持分辨率。
    g_para.height = fmt.fmt.pix_mp.height;
    g_para.num_planes = fmt.fmt.pix_mp.num_planes;
    // printf("Format set: %dx%d, planes: %u\n", g_para.width, g_para.height, g_para.num_planes);

    req.count = VIDIOC_REQBUFS_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(g_para.fd, VIDIOC_REQBUFS, &req))//
    {
        fprintf(stderr, "VIDIOC_REQBUFS error %d, %s\n", errno, strerror(errno));
        return VC_ERR_V4L2_IOCTL;
    }
    if (req.count < 2)//
    {
        fprintf(stderr, "Insufficient buffer memory on %s (got %d)\n", dev_name, req.count);
        return VC_ERR_V4L2_INSUFFCIENT;
    }
    g_para.n_buffers = req.count;
    // printf("Successfully requested %u buffers.\n", g_para.n_buffers);

    g_para.buffers = calloc(req.count, sizeof(*g_para.buffers));
    if (!g_para.buffers)
    {
        fprintf(stderr, "Out of memory\n");
        return VC_ERR_OUTOFMEM;
    }
    
    //查询每个缓冲区的信息并进行内存映射，准备好用户空间访问摄像头数据的地址，使用VIDIOC_QUERYBUF命令查询每个缓冲区的信息，包括物理地址和长度，然后使用mmap函数将这些缓冲区映射到用户空间，以便后续可以直接访问这些缓冲区来读取图像数据。
    for (i = 0; i < g_para.n_buffers; ++i)
    {
        struct v4l2_buffer buf = {0};//定义并初始化一个v4l2_buffer结构体buf,用于存储从摄像头设备读取的图像数据的相关信息。
        struct v4l2_plane planes_query[VIDEO_MAX_PLANES] = {0};//定义一个v4l2_plane数组planes_query,用于存储每个平面的地址和长度信息，VIDEO_MAX_PLANES是V4L2定义的最大平面数量常量，初始化为0.

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = planes_query;
        buf.length = g_para.num_planes;

        if (-1 == xioctl(g_para.fd, VIDIOC_QUERYBUF, &buf))
        {
            fprintf(stderr, "VIDIOC_QUERYBUF error on buffer %u: %d, %s\n", i, errno, strerror(errno));
            return VC_ERR_V4L2_IOCTL;
        }

        g_para.buffers[i].n_planes = g_para.num_planes;
        g_para.buffers[i].planes = calloc(g_para.num_planes, sizeof(struct camera_plane_info));
        if (!g_para.buffers[i].planes) {
            fprintf(stderr, "Out of memory for planes\n");
            return VC_ERR_OUTOFMEM;
        }

        for (unsigned int j = 0; j < g_para.num_planes; ++j) {
            g_para.buffers[i].planes[j].length = buf.m.planes[j].length;
            g_para.buffers[i].planes[j].start = mmap(NULL, buf.m.planes[j].length,
                                            PROT_READ | PROT_WRITE, MAP_SHARED,
                                            g_para.fd, buf.m.planes[j].m.mem_offset);

            if (MAP_FAILED == g_para.buffers[i].planes[j].start)
            {
                fprintf(stderr, "mmap error for buffer %u, plane %u: %d, %s\n", i, j, errno, strerror(errno));
                return VC_ERR_V4L2_MMAP;
            }

            //zengjia
            struct v4l2_exportbuffer expbuf = {0};
            expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            expbuf.index = i;
            expbuf.plane = j;
            expbuf.flags = O_RDWR;
            if (-1 == xioctl(g_para.fd, VIDIOC_EXPBUF, &expbuf)) {
                // 如果导出失败，打印错误但继续，将fd设为无效值
                fprintf(stderr, "WARN: VIDIOC_EXPBUF error on buffer %u, plane %u: %d, %s\n", i, j, errno, strerror(errno));
                g_para.buffers[i].planes[j].fd = -1;
            } else {
                g_para.buffers[i].planes[j].fd = expbuf.fd;
            }
        }
    }

    for (i = 0; i < g_para.n_buffers; ++i)
    {
        struct v4l2_buffer buf = {0};
        struct v4l2_plane planes_q[VIDEO_MAX_PLANES] = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = planes_q;
        buf.length = g_para.num_planes;
        if (-1 == xioctl(g_para.fd, VIDIOC_QBUF, &buf))
        {
            fprintf(stderr, "VIDIOC_QBUF error %d, %s\n", errno, strerror(errno));
            return VC_ERR_V4L2_IOCTL;
        }
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (-1 == xioctl(g_para.fd, VIDIOC_STREAMON, &type))
    {
        fprintf(stderr, "VIDIOC_STREAMON error %d, %s\n", errno, strerror(errno));
        return VC_ERR_V4L2_IOCTL;
    }

    g_para.is_init = 1;
    return VC_SUCCESS;
}

static int V4l2CameraStart(void)
{
    int ret = VC_ERR_MAX;

    if(!g_para.is_init)
    {
        printf("g_V4l2Camera not init,ignore it.\n");
        return VC_ERR_NOT_INIT;
    }
    if(g_para.camera_pid_on)
    {
        printf("g_V4l2Camera already started,ignore it.\n");
        return VC_SUCCESS;
    }

    g_para.camera_pid_on = 1;
    ret = pthread_create(&g_para.camera_pid, NULL, CameraLoopProcess, NULL);
    if(ret)
    {
        printf("pthread_create failed(%d)\n", ret);
        g_para.camera_pid_on = 0;
        return VC_ERR_PTHREAD_CREATE;
    }
    return VC_SUCCESS;
}

static int V4l2CameraStop(void)
{
    if(g_para.camera_pid_on)
    {
        g_para.camera_pid_on = 0;
        pthread_join(g_para.camera_pid, NULL);
    }
    return VC_SUCCESS;
}

static int V4l2CameraUninit(void)
{
    unsigned int i, j;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    V4l2CameraStop();

    if(g_para.is_init)
    {
        if (-1 == xioctl(g_para.fd, VIDIOC_STREAMOFF, &type))
        {
            fprintf(stderr, "v4l2 VIDIOC_STREAMOFF error %d, %s\n", errno, strerror(errno));
        }

        for (i = 0; i < g_para.n_buffers; ++i)
        {
            for (j = 0; j < g_para.buffers[i].n_planes; ++j) {
                if (-1 == munmap(g_para.buffers[i].planes[j].start, g_para.buffers[i].planes[j].length))
                {
                    fprintf(stderr, "v4l2 munmap error %d, %s\n", errno, strerror(errno));
                }
                //zengjia
                if (g_para.buffers[i].planes[j].fd >= 0) { close(g_para.buffers[i].planes[j].fd); }
            }
            free(g_para.buffers[i].planes);
        }
        free(g_para.buffers);

        if (-1 == close(g_para.fd))
        {
            fprintf(stderr, "v4l2 close error %d, %s\n", errno, strerror(errno));
        }
        g_para.fd = -1;
        g_para.is_init = 0;
    }
    return VC_SUCCESS;
}

static void V4l2CameraSetCallback(v4l2camera_callback_t callback)
{
    g_para.callback = callback;
}

static int V4l2CameraReStart(void)
{
    if(g_para.is_init)
    {
        V4l2CameraUninit();
        V4l2CameraInit(g_para.device, g_para.width, g_para.height, g_para.fps, g_para.format);
        V4l2CameraStart();
    }
    else
        printf("camera need init first!\n");
    return 0;
}

static V4l2Camera_t g_V4l2Camera =
{
    .init = V4l2CameraInit,
    .uninit = V4l2CameraUninit,
    .start = V4l2CameraStart,
    .restart = V4l2CameraReStart,
    .stop = V4l2CameraStop,
    .SetCallback = V4l2CameraSetCallback,
};

V4l2Camera_t* GetV4l2CameraInstance(void)
{
    return &g_V4l2Camera;
}
