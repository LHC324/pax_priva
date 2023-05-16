#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <fal.h>
#include <dfs_fs.h>
#include "dfs_file.h" /* 当需要使用文件操作时，需要包含这个头文件 */
#include "fcntl.h"
#include "unistd.h"
#include "user_photosynthesis.h"

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "user_file"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define USER_FILE_FORMAT_MAX 64
#define USER_FILE_PATH_MAX 64
#define USER_FILE_NAME_MAX 64

typedef enum
{
    user_sys_poto,
    user_sys_eneygy,
    user_sys_climate,
    user_sys_water,
    user_sys_irrigate,
    user_sys_max,
} ueser_file_sys_type;

struct user_file
{
    char format[USER_FILE_FORMAT_MAX];
    char path[USER_FILE_PATH_MAX];
    char name[USER_FILE_NAME_MAX];
    ueser_file_sys_type sys_type;
    void *user;
};

struct user_file user_file_param[user_sys_max] = {
    {
        .sys_type = user_sys_poto,
        .format = "",
        .path = "/sys/poto.bin",
        .user = &poto_system,
    },
};

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

    /*fal层使用：https://github.com/RT-Thread-packages/fal/blob/master/README_ZH.md#falflash-%E6%8A%BD%E8%B1%A1%E5%B1%82*/
    fal_init();
    /* 生成 mtd 设备 */
    mtd_dev = fal_mtd_nor_device_create(FS_PARTITION_NAME);
    if (mtd_dev == RT_NULL)
    {
        LOG_E("Can't create a mtd device on '%s' partition.", FS_PARTITION_NAME);
    }
    else
    {
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
            }
        }
    }
#undef FS_PARTITION_NAME
    return 0;
}
INIT_ENV_EXPORT(mount_file_system);

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
#define USER_FILE_BUF_SIZE 512U
    int fd;
    int size;
    // FILE *fp = NULL;

    RT_ASSERT(user);

    // if ((fp = fopen(user->path, "rb+")) == NULL)
    // {
    //     puts("Open file[%s] failed\n", user->);
    //     return;
    // }
    // else
    // {
    //     puts("Open file success\n");
    // }

    // char *buf = (char *)rt_malloc(512);

    /* 以创建和读写模式打开 /text.txt 文件，如果该文件不存在则创建该文件*/
    fd = open(user->path, O_WRONLY | O_CREAT);
    if (fd < 0)
    {
        rt_kprintf("open failed.\n");
        return;
    }

    switch (user->sys_type)
    {
    case user_sys_poto:
    {
        // #define POTO_PARAM_FORMAT
        // struct potos *poto = (struct potos *)user->user;
        // RT_ASSERT(poto);

        // 写入结构体到文件
        // fwrite(poto, sizeof(struct potos), 1, fp);

        // size = rt_snprintf(NULL, 0, user->format, );

        size = sizeof(struct potos);
    }
    break;
    default:
        return;
    }

    write(fd, user->user, size);

    rt_kprintf("Write done.\n");
    LOG_HEX(user->path, 16U, user->user, size);
    close(fd);
    // rt_free(buf);
#undef USER_FILE_BUF_SIZE
}

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
