#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <fal.h>
#include <dfs_fs.h>
#include "dfs_file.h" /* 当需要使用文件操作时，需要包含这个头文件 */
#include "fcntl.h"
#include "unistd.h"
#include "user_photosynthesis.h"
#include "user_climate.h"
#include "user_water_room.h"

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "user.file"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define USER_FILE_FORMAT_MAX 64
#define USER_FILE_PATH_MAX 64
#define USER_FILE_NAME_MAX 64

// typedef enum
// {
//     user_sys_poto,
//     user_sys_eneygy,
//     user_sys_climate,
//     user_sys_water,
//     user_sys_irrigate,
//     user_sys_max,
// } ueser_file_sys_type;

struct user_file
{
    char format[USER_FILE_FORMAT_MAX];
    char path[USER_FILE_PATH_MAX];
    char name[USER_FILE_NAME_MAX];
    // ueser_file_sys_type sys_type;
    void *user;
    uint32_t size;
};

struct user_file user_file[] = {
    {
        // .sys_type = user_sys_poto,
        .format = "",
        .path = "/sys/poto.bin",
        .user = &poto_system,
        .size = sizeof(poto_system),
    },
    {
        // .sys_type = user_sys_climate,
        .format = "",
        .path = "/sys/climate.bin",
        .user = &clim_system,
        .size = sizeof(clim_system),
    },
    {
        // .sys_type = user_sys_water,
        .format = "",
        .path = "/sys/water.bin",
        .user = &water_system,
        .size = sizeof(water_system),
    },
};

void *file_mail_msg[ sizeof(user_file) / sizeof(user_file[0])];

/* 邮箱控制块 */
static struct rt_mailbox file_mb;
/* 用于放邮件的内存池 */
static char file_mb_pool[128];

/**
 * @brief	检查目标文件是否存在
 * @details
 * @note    cap'n proto 可行性分析：https://noodlefighter.com/posts/2053/
 * @param	name 文件名
 * @param   info 文件信息
 * @retval  none
 */
static int file_exist_check(const char *name, struct stat *info)
{
    int ret;

    if ((NULL == name) || (NULL == info))
        return RT_ERROR;

    ret = stat(name, info);

    if (ret == RT_EOK)
    {
        rt_kprintf("\nname\t\tbyte\r\n%s\t%d\n",
                   name, info->st_size);
    }
    else
        rt_kprintf("\nfile[%s] not fonud ^_^.\n");

    return ret;
}

/**
 * @brief	打开目标文件
 * @details
 * @note    
 * @param	name 文件名
 * @param   info 文件信息
 * @retval  fd
 */
static int file_open_check(struct user_file *user)
{
    struct stat file_stat;
    int fd;

    if (file_exist_check(user->path, &file_stat))
    {
        /* 以创建和读写模式打开xx文件，如果该文件不存在则创建该文件*/
        fd = open(user->path, O_WRONLY | O_CREAT);
    }
    else
        /* 以读写模式打开 /text.txt 文件，如果该文件不存在则创建该文件*/
        fd = open(user->path, O_WRONLY);

    if (fd < 0)
    {
        rt_kprintf("file[%s]:open failed.\n", user->path);
    }

    return fd;
}

/**
 * @brief	缓冲区写回文件
 * @details
 * @note    cap'n proto 可行性分析：https://noodlefighter.com/posts/2053/
 * @param	fd    文件描述符
 * @param   wptr  结构数据指针
 * @retval  none
 */
// static int file_buf_write_back(int fd, const char *wptr)
// {
//     off_t offset;

//     if (fd < 0)
//     {
//         rt_kprintf("fd[%d] error ^_^.\n", fd);
//         return RT_ERROR;
//     }

//     if(NULL == wptr)
//      {
//         rt_kprintf("wptr error.\n");
//         return RT_ERROR;
//     }

//     offset = lseek(fd, 0, SEEK_SET); // 重新定位文件读写指针到开头

//     if (offset < 0)
//     {
//         rt_kprintf("lseek failed.\n");
//         return RT_ERROR;
//     }

//     write(fd, write_buf, sizeof(write_buf));
//     close(fd);

//     return RT_EOK;
// }

/**
 * @brief	从文件读取数据到结构
 * @details
 * @note    ISO/LIBC库文件操作接口：https://blog.csdn.net/unsv29/article/details/105677625
 * @note    linux 下结构体写入文件：https://blog.csdn.net/qq1140920745/article/details/109658291
 * @param	fd    文件描述符
 * @param   wptr  结构数据指针
 * @retval  none
 */
static void file_read_to_struct(struct user_file *user)
{
    int fd;

    RT_ASSERT(user);

    fd = file_open_check(user);

    if (fd < 0)
        return;

    /* 以只读模式打开 xx 文件 */
    // fd = open(user->path, O_RDONLY);

    if (read(fd, user->user, user->size) < 0)
    {
        rt_memset(user->user, 0x00, user->size);
        rt_kprintf("read failed.\n");
    }
    else
        LOG_HEX(user->path, 16U, user->user, user->size);

    close(fd);
}

/**
 * @brief	编码目标结构体到文件
 * @details
 * @note    ISO/LIBC库文件操作接口：https://blog.csdn.net/unsv29/article/details/105677625
 * @note    linux 下结构体写入文件：https://blog.csdn.net/qq1140920745/article/details/109658291
 * @param	fd    文件描述符
 * @param   wptr  结构数据指针
 * @retval  none
 */
static void file_encode_struct_to_file(struct user_file *user)
{
    int fd;

    RT_ASSERT(user);

    fd = file_open_check(user);

    if (fd < 0)
        return;

    if (write(fd, user->user, user->size) < 0)
    {
        rt_kprintf("write failed.\n");
    }

    rt_kprintf("Write done.\n");
    LOG_HEX(user->path, 16U, user->user, user->size);
    close(fd);
}

/**
 * @brief	结构数据初始化
 * @details
 * @note    cap'n proto 可行性分析：https://noodlefighter.com/posts/2053/
 * @param	fd    文件描述符
 * @param   wptr  结构数据指针
 * @retval  none
 */
static int file_struct_init(void)
{
    struct user_file *cur_user;
    void *mail_msg = file_mail_msg;

//    for (cur_user = user_file;
//         cur_user < user_file + sizeof(user_file) / sizeof(user_file[0]);
//         ++cur_user, mail_msg += sizeof(struct user_file))
//    {
//        file_read_to_struct(cur_user);
//        mail_msg = cur_user; //地址存到邮件类型列表中
//    }

    return 0;
}

/**
 * @brief   文件操作线程
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
static void file_ops_thread_entry(void *parameter)
{
    struct user_file *mail = RT_NULL;
    for (;;)
    {
        /* 从邮箱中收取邮件 */
        if (rt_mb_recv((rt_mailbox_t)parameter, (rt_ubase_t *)&mail, RT_WAITING_FOREVER) == RT_EOK)
        {
            rt_kprintf("get a mail from mailbox, the content:%#x\n", mail);
            if (mail)
                file_encode_struct_to_file(mail);
        }
    }
}

/**
 * @brief	挂载文件系统到rt_thread
 * @details
 * @note    littlefs文件系统使用教程：https://club.rt-thread.org/ask/article/a5c8b007eed2584e.html
 * @param	None
 * @retval  none
 */
static int mount_file_system(void)
{
/* 定义要使用的分区名字 */
#define FS_PARTITION_NAME "filesystem"
    struct rt_device *mtd_dev = RT_NULL;
    rt_err_t result;

    /*fal层使用：https://github.com/RT-Thread-packages/fal/blob/master/README_ZH.md#falflash-%E6%8A%BD%E8%B1%A1%E5%B1%82*/
    fal_init();
    /* 生成 mtd 设备 */
    mtd_dev = fal_mtd_nor_device_create(FS_PARTITION_NAME);
    if (mtd_dev == RT_NULL)
    {
        LOG_E("Can't create a mtd device on '%s' partition.", FS_PARTITION_NAME);
        return -1;
    }
    /* 挂载 littlefs */
    if (dfs_mount(FS_PARTITION_NAME, "/", "lfs", 0, 0) == 0)
    {
        LOG_I("Filesystem initialized!");
    }
    else
    {
        /* 格式化文件系统 */
        dfs_mkfs("lfs", FS_PARTITION_NAME);
        /* 挂载 littlefs */
        if (dfs_mount("filesystem", "/", "lfs", 0, 0) == 0)
        {
            LOG_I("Filesystem initialized!");
        }
        else
        {
            LOG_E("Failed to initialize filesystem!");
            return -1;
        }
    }

    file_struct_init();

    /* 初始化一个 mailbox */
    result = rt_mb_init(&file_mb,
                        "file_mbt",               /* 名称是 mbt */
                        &file_mb_pool[0],         /* 邮箱用到的内存池是 mb_pool */
                        sizeof(file_mb_pool) / 4, /* 邮箱中的邮件数目，因为一封邮件占 4 字节 */
                        RT_IPC_FLAG_FIFO);        /* 采用 FIFO 方式进行线程等待 */
    if (result != RT_EOK)
    {
        rt_kprintf("init file mailbox failed.\n");
        return -1;
    }

    rt_thread_t tid = rt_thread_create(
        "file_ops",
        file_ops_thread_entry,
        &file_mb,
        512, 0x05, 20);

    RT_ASSERT(tid != RT_NULL);

    rt_thread_startup(tid);

#undef FS_PARTITION_NAME
    return 0;
}
INIT_ENV_EXPORT(mount_file_system);


#ifdef FINSH_USING_MSH
#include <finsh.h>

/**
 * @brief   文件系统测试
 * @details
 * @param	None
 * @retval  None
 */
static void file_test(void)
{
    struct user
    {
        char name[8];
        int age;
        float score;
    };
	
	int fd;

    struct user me = {"lhc", 25, 96.6f}, me1;

    /* 以创建和读写模式打开 /text.bin 文件，如果该文件不存在则创建该文件*/
    fd = open("/text.bin", O_WRONLY | O_CREAT);
    if (fd >= 0)
    {
        write(fd, &me, sizeof(struct user));
        close(fd);
        rt_kprintf("Write done.\n");
    }

    /* 以只读模式打开 /text.bin 文件 */
    fd = open("/text.bin", O_RDONLY);
    if (fd >= 0)
    {
        read(fd, &me1, sizeof(struct user));
        close(fd);
        rt_kprintf("name\tage\tscore\n%s\t%d\t%.1f\n", me1.name, me1.age, me1.score);
        LOG_HEX("/text.bin", 16U, (uint8_t *)&me1, sizeof(me1));
    }
}
MSH_CMD_EXPORT(file_test, File system test structure data writing.);

#endif
